import numpy as np
import re
import sys
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

def parse_eus_txt(file_path):
    times = []
    positions = []
    pattern = re.compile(r"([\d\.]+):\s+#f\(([\d\.\-\s]+)\)")

    with open(file_path, 'r') as f:
        for line in f:
            match = pattern.search(line)
            if match:
                t = float(match.group(1))
                coords = [float(x) for x in match.group(2).split()]
                if len(coords) == 3:
                    times.append(t)
                    positions.append(coords)
    
    return np.array(times), np.array(positions) * 0.001

def calculate_ros_style_derivatives(t, pos):
    """
    CubicSplineの微分計算を修正しました。
    cs(t, nu=1) で1階微分（速度）、nu=2 で2階微分（加速度）を取得します。
    """
    # 3次スプライン補間
    cs = CubicSpline(t, pos, axis=0, bc_type='natural')
    
    # 補間曲線上の位置、速度、加速度を算出
    # nu引数で微分の次数を指定します
    smoothed_p = cs(t)          # 位置
    v_vec = cs(t, nu=1)         # 速度ベクトル [m/s]
    a_vec = cs(t, nu=2)         # 加速度ベクトル [m/s^2]

    # ノルム（ベクトルの大きさ）を計算
    v_norm = np.linalg.norm(v_vec, axis=1)
    a_norm = np.linalg.norm(a_vec, axis=1)
    
    return smoothed_p, v_norm, a_norm

def main():
    if len(sys.argv) < 2:
        print("Usage: python analyze_ros_control_trajectory.py <input_txt_file>")
        return

    input_path = sys.argv[1]
    t, pos = parse_eus_txt(input_path)

    # 重複時刻の除去
    t, unique_indices = np.unique(t, return_index=True)
    pos = pos[unique_indices]

    if len(t) < 3:
        print("Error: データポイントが不足しています。")
        return

    # 微分計算
    smoothed_p, v_norm, a_norm = calculate_ros_style_derivatives(t, pos)

    max_acc = np.max(a_norm)
    g = 9.80665
    ratio = (max_acc / g) * 100

    print("\n" + "="*50)
    print(f"【解析結果】")
    print(f"サンプリング点数: {len(t)}")
    print(f"計測時間: {t[-1] - t[0]:.3f} [s]")
    print("-" * 50)
    print(f"最大加速度 (a_max):  {max_acc:.6f} [m/s^2]")
    print(f"重力加速度 (g):      {g:.6f} [m/s^2]")
    print(f"慣性項の重力比 (a/g): {ratio:.2f} %")
    print("="*50 + "\n")

    # グラフ描画
    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

    colors = ['tab:red', 'tab:green', 'tab:blue']
    labels = ['X [m]', 'Y [m]', 'Z [m]']
    for i in range(3):
        axs[0].scatter(t, pos[:, i], color=colors[i], alpha=0.3, s=15)
        axs[0].plot(t, smoothed_p[:, i], color=colors[i], label=labels[i])
    axs[0].set_ylabel('Position [m]')
    axs[0].set_title('Centroid Trajectory (Cubic Spline)')
    axs[0].grid(True, alpha=0.3)
    axs[0].legend()

    axs[1].plot(t, v_norm, color='black', label='Velocity Norm [m/s]')
    axs[1].set_ylabel('Velocity [m/s]')
    axs[1].grid(True, alpha=0.3)
    axs[1].legend()

    axs[2].plot(t, a_norm, color='darkviolet', label='Acceleration Norm [m/s^2]')
    axs[2].axhline(y=max_acc, color='red', linestyle=':', label=f'Max: {max_acc:.4f}')
    axs[2].set_ylabel('Acceleration [m/s^2]')
    axs[2].set_xlabel('Time [s]')
    axs[2].grid(True, alpha=0.3)
    axs[2].legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
