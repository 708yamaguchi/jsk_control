import subprocess
import os
import sys
import concurrent.futures
from threading import Lock

# 表示用のカラーコード
GREEN = '\033[92m'
RED = '\033[91m'
BOLD = '\033[1m'
END = '\033[0m'

# 並列実行中の表示が混ざらないようにするためのロック
print_lock = Lock()

def run_single_trial(task_name, seed_id):
    """
    1つのseedを実行するワーカー関数。
    roseusをサブプロセスとして実行し、成否を返す。
    """
    lisp_command = f"(check-success-cl \"{task_name}\" {seed_id})"
    cmd = ["roseus", "main.l", lisp_command]

    try:
        # roseusの出力を完全に抑制
        process = subprocess.run(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        # 終了ステータス0なら成功
        return seed_id, (process.returncode == 0)
    except Exception as e:
        return seed_id, False

def evaluate_task_parallel(task_name):
    total_trials = 100
    success_count = 0
    completed_count = 0

    # CPUコア数の取得
    num_cores = os.cpu_count()
    if num_cores is None:
        num_cores = 1

    print(f"\n{BOLD}## Parallel Evaluation: {task_name}{END}")
    print(f"Using {num_cores} CPU cores.")
    print("-" * 60)
    print(f"{'Order':<8} | {'Seed':<8} | {'Status':<12} | {'Success Rate'}")
    print("-" * 60)

    # ThreadPoolExecutorを使用してサブプロセスを並列管理
    # (roseus自体が別プロセスなのでThread管理が効率的です)
    with concurrent.futures.ThreadPoolExecutor(max_workers=num_cores) as executor:
        # 全てのseed(0-99)の実行を予約
        futures = [executor.submit(run_single_trial, task_name, seed) for seed in range(total_trials)]

        # 終わったものから順番に処理
        for future in concurrent.futures.as_completed(futures):
            seed_id, is_success = future.result()

            with print_lock:
                completed_count += 1
                if is_success:
                    success_count += 1
                    status = f"{GREEN}SUCCESS{END}"
                else:
                    status = f"{RED}FAILED{END} "

                rate = (success_count / completed_count) * 100

                # 並列実行なので、完了した順番（Order）とSeed IDを分けて表示
                print(f"[{completed_count:03d}/100] | Seed {seed_id:02d} | {status:<20} | {rate:6.1f}% ({success_count}/{completed_count})")

    # --- 最終結果の表示 ---
    final_rate = (success_count / total_trials) * 100
    print("\n" + "="*60)
    print(f"{BOLD}FINAL REPORT: {task_name}{END}")
    print("="*60)
    print(f"Total Trials   : {total_trials}")
    print(f"Successes      : {GREEN}{success_count}{END}")
    print(f"Failures       : {RED}{total_trials - success_count}{END}")
    print(f"Final Success Rate : {BOLD}{final_rate:.2f}%{END}")
    print("="*60 + "\n")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        target_task = sys.argv[1]
        evaluate_task_parallel(target_task)
    else:
        print("Usage: python3 script.py <task_name>")
        print("Example: python3 script.py microwave")
