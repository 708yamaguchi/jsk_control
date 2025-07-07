import subprocess
import sys
import os
import signal
from multiprocessing import Pool, Manager

# もしゾンビプロセスが残っていた場合に全てkillするコマンド
# ps aux | grep "roseus" | grep -v grep | awk '{print $2}' | xargs kill

# グローバル変数としてプロセスプールを保持
# シグナルハンドラからアクセスするために必要
pool = None

def signal_handler(sig, frame):
    """
    Ctrl+Cなどのシグナルを捕捉したときに呼び出される関数
    """
    global pool
    print("\n中断リクエストを受け取りました。すべてのプロセスを終了します...", file=sys.stderr)
    if pool:
        # ワーカプロセスのタスクを強制終了
        pool.terminate()
        # プロセスの終了を待つ
        pool.join()
    sys.exit(1)


def run_grid_search_multiprocess_single_output(input_file='input.txt', output_file='output.txt', num_processes=None):
    """
    input.txtからセミコロン区切りのパラメータを読み込み、
    グリッドサーチをマルチプロセスで実行して結果を単一の出力ファイルに書き込む。
    (シグナルハンドリング対応済み)
    """
    global pool
    print(f"'{input_file}' からパラメータを読み込んでいます...")

    try:
        with open(input_file, 'r', encoding='utf-8') as f_in:
            param_lines = f_in.readlines()
    except FileNotFoundError:
        print(f"エラー: '{input_file}' が見つかりません。", file=sys.stderr)
        sys.exit(1)

    total_lines = len(param_lines)
    # valid_linesとその元の行番号を保存するリスト
    valid_lines = []
    valid_line_indices = []
    for i, line in enumerate(param_lines):
        # 行の'#'以前を取り出して、空でなければパラメータ行であると判定
        stripped_line = line.strip().split('#')[0]
        if stripped_line:
            valid_lines.append(stripped_line)
            valid_line_indices.append(i)

    if num_processes is None:
        num_processes = os.cpu_count() or 1

    print(f"--- {num_processes}個のプロセスで{len(valid_lines)}個の条件をグリッドサーチし、結果を'{output_file}' に出力します ---")

    # Managerを作成し、共有ロックを管理
    with Manager() as manager:
        shared_lock = manager.Lock()

        # 出力ファイルを初期化（'w'モードで空にする）
        with open(output_file, 'w', encoding='utf-8') as f_out:
            pass

        tasks = [(i, line, total_lines, output_file, shared_lock)
                 for i, line in zip(valid_line_indices, valid_lines)]

        try:
            # プロセスプールを作成し、グローバル変数に格納
            pool = Pool(processes=num_processes)
            # mapはiterableの各要素を関数に渡す
            pool.map(process_parameter_line_with_robust_subprocess, tasks)

            # 正常終了時はプールを閉じる
            pool.close()
            pool.join()

        except Exception as e:
            print(f"メインプロセスで予期せぬエラーが発生しました: {e}", file=sys.stderr)
            # エラー発生時はプールを強制終了
            if pool:
                pool.terminate()
                pool.join()

    print("-" * 60)
    print(f"処理が完了しました。すべての結果は '{output_file}' に書き込まれました。")


def process_parameter_line_with_robust_subprocess(line_info):
    """
    単一のパラメータ行を処理し、結果を共有ロックを使って単一のファイルに書き込む関数。
    サブプロセスを新しいプロセスグループで起動し、確実に終了させる。
    """
    line_index, line, total_lines, output_file_path, lock = line_info
    line_index += 1  # 1-indexedに変換

    params = line.split(';')
    if len(params) != 5:
        sys.stderr.write(f"  警告 (プロセスID: {os.getpid()}): {line_index}行目のフォーマットが不正です。スキップします: {line}\n")
        sys.stderr.flush()
        return

    random_angle_vector_seed, kin_scale, base_size_scale, fix_variant, fix_pad_diameter = [p.strip() for p in params]
    lisp_command = (
        f"(progn "
        # f" (main (list *faucet-task*)"
        # f" (main (list *faucet-wall-task*)"
        # f" (main (list *ih-task*)"
        # f" (main (list *microwave-task*)"
        f" (main (list *wipe-ih-wall-task*)"
        f" :random-angle-vector-seed {random_angle_vector_seed}"
        f" :kin-scale-list {kin_scale}"
        f" :base-size-scale-list {base_size_scale}"
        f" :fix-variant-joint-list {fix_variant}"
        f" :fix-pad-diameter-list {fix_pad_diameter}"
        f" :debug-view nil"
        f")"
        f" (exit)"
        f")"
    )
    command_list = ['roseus', 'main.l', lisp_command]

    print("-" * 60)
    print(f"実行中 (プロセスID: {os.getpid()}, 元の行: {line_index}/{total_lines})")
    print(f" random-angle-vector-seed: {random_angle_vector_seed}")
    print(f" kin-scale: {kin_scale}")
    print(f" base-size: {base_size_scale}")
    print(f" fix-joint: {fix_variant}")
    print(f" fix-pad-diameter: {fix_pad_diameter}")

    proc = None
    try:
        # Popenでサブプロセスを開始。start_new_session=Trueで新しいプロセスグループを作成。
        proc = subprocess.Popen(
            command_list,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding='utf-8',
            start_new_session=True  # ★重要: これでプロセスグループが作られる
        )
        # サブプロセスの終了を待つ
        stdout, stderr = proc.communicate()

        # ロックを取得してファイルに書き込む
        with lock:
            with open(output_file_path, 'a', encoding='utf-8') as f_out:
                f_out.write(f"==== PARAMETERS START (Input Line: {line_index}) ====\n")
                f_out.write(f"Parameters: {line}\n")
                f_out.write(f"==========================================\n")
                if proc.returncode == 0:
                    f_out.write(stdout)
                    print(f"プロセスID: {os.getpid()}が成功しました。")
                else:
                    # エラーの場合
                    error_message = (
                        f"Command failed with exit code {proc.returncode}\n"
                        f"Stdout:\n{stdout}\n"
                        f"Stderr:\n{stderr}\n"
                    )
                    f_out.write(error_message)
                    sys.stderr.write(f"エラー (プロセスID: {os.getpid()}, 行: {line_index}): コマンド実行に失敗しました。詳細は '{output_file_path}' を確認してください。\n")
                    sys.stderr.flush()

                f_out.write(f"\n==== PARAMETERS END (Input Line: {line_index}) ====\n")
                f_out.write("\n\n")

    except Exception as e:
        # このワーカープロセス自体で起きた予期せぬエラー
        sys.stderr.write(f"予期せぬエラー (プロセスID: {os.getpid()}, 行: {line_index}): {e}\n")
        sys.stderr.flush()
    finally:
        # 重要: 正常終了・エラーを問わず、サブプロセスがまだ生きていればkillする
        if proc and proc.poll() is None:
            print(f"プロセスグループ {proc.pid} をクリーンアップします...", file=sys.stderr)
            try:
                # プロセスグループ全体にSIGTERMシグナルを送信
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except ProcessLookupError:
                # プロセスが既に存在しない場合は何もしない
                pass

if __name__ == '__main__':
    # ROS環境のチェック
    ros_package_path = os.getenv('ROS_PACKAGE_PATH')
    if not (ros_package_path and 'eus_qp' in ros_package_path):
        print('Cannot find eus_qp package. Use source command', file=sys.stderr)
        sys.exit(1)

    # 既存の出力ファイルを削除
    output_filename = 'output.txt'
    if os.path.exists(output_filename):
        os.remove(output_filename)
        print(f"既存の '{output_filename}' を削除しました。")

    # シグナルハンドラを設定 (Ctrl+C と kill コマンドに対応)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    run_grid_search_multiprocess_single_output(output_file=output_filename)
