import subprocess
import sys
import os
from multiprocessing import Pool, Manager, Lock


def run_grid_search_multiprocess_single_output(input_file='input.txt', output_file='output.txt', num_processes=None):
    """
    input.txtからセミコロン区切りのパラメータを読み込み、
    グリッドサーチをマルチプロセスで実行して結果を単一の出力ファイルに書き込む。
    """
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
        num_processes = os.cpu_count() or 1 # CPUコア数を取得、取得できない場合は1

    print(f"--- {num_processes}個のプロセスで{len(valid_line_indices)}個の条件をグリッドサーチし、結果を'{output_file}' に出力します ---")

    # Managerを作成し、共有オブジェクトを管理する
    with Manager() as manager:
        # Manager経由で共有ファイルオブジェクトを作成 (実際にはファイル名を共有し、各プロセスでオープン)
        # より直接的な方法として、Managerは共有可能な型 (リスト、辞書、Value、Array) を提供しますが、
        # ファイルオブジェクト自体を直接共有することはできません。
        # 代わりに、Managerを使って Lock を共有し、各プロセスが同じファイルにアクセスする際に同期を取ります。

        # ロックオブジェクトをマネージャーから取得
        shared_lock = manager.Lock()

        # 各プロセスが書き込むファイルをメインプロセスで開く
        # 'w' モードで開始し、既存の内容をクリア
        with open(output_file, 'w', encoding='utf-8') as f_out:
            # f_out は Manager の管理下にはないが、ロックにより排他制御される
            # ここではダミーとしてf_outを渡すのではなく、プロセス内でファイルを開く
            pass # 初期化のために開いてすぐ閉じるか、後で'a'モードで開く

        # タスクリストを作成 (共有ファイルオブジェクトは直接渡せないため、ロックのみ渡す)
        # ここでの工夫は、shared_file を Manager.list() のようなものにしない点。
        # ファイル操作は低レベルなため、Manager が直接サポートするオブジェクトではない。
        # 各プロセスで output_file を 'a' (追記) モードで開き、共有ロックを使って排他制御する。
        tasks = [(i, line, total_lines, output_file, shared_lock)
                 for i, line in zip(valid_line_indices, valid_lines)]

        # プロセスプールを作成し、タスクを実行
        with Pool(processes=num_processes) as pool:
            # mapはiterableの各要素を関数に渡す。この場合、ファイルパスと共有ロックを渡す
            pool.map(process_parameter_line_to_shared_file_with_reopen, tasks) # 関数名を変更

    print("-" * 60)
    print(f"処理が完了しました。すべての結果は '{output_file}' に書き込まれました。")


# 各プロセスでファイルを開く
def process_parameter_line_to_shared_file_with_reopen(line_info):
    """
    単一のパラメータ行を処理し、結果を共有ロックを使って単一のファイルに書き込む関数
    (各プロセスがファイルを開き直す)
    """
    line_index, line, total_lines, output_file_path, lock = line_info

    # input.txt の実際の行番号 (0-indexed -> 1-indexed)
    line_index += 1
    line = line.strip()
    if not line or line.startswith('#'):
        return

    params = line.split(';')
    if len(params) != 4:
        sys.stderr.write(f"  警告 (プロセスID: {os.getpid()}): {line_index+1}行目のフォーマットが不正です（パラメータが4つではありません）。スキップします: {line}\n")
        sys.stderr.flush()
        return

    random_angle_vector_seed, kin_scale, base_size_scale, fix_variant = [p.strip() for p in params]
    lisp_command = (
        f"(progn "
        f" (main (list *faucet-task*)"
        # f" (main (list *faucet-task* *microwave-task* *table-task* *ih-task*)"
        f" :pad-diameter 80"
        f" :random-angle-vector-seed {random_angle_vector_seed}"
        f" :kin-scale-list {kin_scale}"
        f" :base-size-scale-list {base_size_scale}"
        f" :fix-variant-joint-list {fix_variant}"
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

    try:
        result = subprocess.run(
            command_list,
            capture_output=True,
            text=True,
            encoding='utf-8',
            check=True
        )

        # ロックを取得してからファイルに書き込む
        # 各子プロセスがファイルを追記モード ('a') で開く
        with lock:
            with open(output_file_path, 'a', encoding='utf-8') as f_out:
                f_out.write(f"==== PARAMETERS START (Input Line: {line_index}) ====\n")
                f_out.write(f"Parameters: {line}\n")
                f_out.write(f"==========================================\n")
                f_out.write(result.stdout)
                f_out.write(f"\n==== PARAMETERS END (Input Line: {line_index}) ====\n")
                f_out.write("\n\n")
        print(f"プロセスID: {os.getpid()}が成功しました。")

    except subprocess.CalledProcessError as e:
        with lock:
            with open(output_file_path, 'a', encoding='utf-8') as f_out:
                f_out.write(f"==== PARAMETERS START (Input Line: {line_index}) ====\n")
                f_out.write(f"Parameters: {line}\n")
                f_out.write(f"==========================================\n")
                f_out.write(f"Command failed with exit code {e.returncode}\n")
                f_out.write(f"Stdout:\n{e.stdout}\n")
                f_out.write(f"Stderr:\n{e.stderr}\n")
                f_out.write(f"\n==== PARAMETERS END (Input Line: {line_index}) (ERROR) ====\n")
                f_out.write("\n\n")
        sys.stderr.write(f"エラー (プロセスID: {os.getpid()}): {e}. 詳細は '{output_file_path}' を確認してください。\n")
        sys.stderr.flush()
    except Exception as e:
        sys.stderr.write(f"予期せぬエラー (プロセスID: {os.getpid()}): {e}\n")
        sys.stderr.flush()


if __name__ == '__main__':
    ros_package_path = os.getenv('ROS_PACKAGE_PATH')
    if not (ros_package_path and 'eus_qp' in ros_package_path):
        print('Cannot find eus_qp package. Use source command')
        exit()

    if os.path.exists('output.txt'):
        os.remove('output.txt')
        print("既存の 'output.txt' を削除しました。")

    run_grid_search_multiprocess_single_output()
