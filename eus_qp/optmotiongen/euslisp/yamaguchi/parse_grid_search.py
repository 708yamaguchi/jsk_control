import argparse
import matplotlib.pyplot as plt
import numpy as np
import re


def parse_grid_search_output(output_file='output.txt'):
    """
    グリッドサーチの出力ファイル (output.txt) をパースし、
    入力行番号ごとに結果を辞書として返す。
    """
    results = {}
    current_line_number = None
    current_block_lines = []

    # 正規表現で開始/終了マーカーと行番号を抽出
    start_pattern = re.compile(r"==== PARAMETERS START \(Input Line: (\d+)\) ====")
    end_pattern = re.compile(r"==== PARAMETERS END \(Input Line: (\d+)\)( \(ERROR\))? ====")

    try:
        with open(output_file, 'r', encoding='utf-8') as f_in:
            for line in f_in:
                start_match = start_pattern.match(line)
                end_match = end_pattern.match(line)

                if start_match:
                    # 新しいブロックの開始
                    current_line_number = int(start_match.group(1))
                    current_block_lines = [line] # 開始マーカーもブロックに含める
                elif end_match and current_line_number is not None:
                    # ブロックの終了
                    current_block_lines.append(line)
                    results[current_line_number] = "".join(current_block_lines)
                    current_line_number = None # 次のブロックに備えてリセット
                    current_block_lines = []
                elif current_line_number is not None:
                    # ブロック内の行を追加
                    current_block_lines.append(line)

        # ファイルの最後に終了マーカーがない場合の処理（もしあれば）
        if current_line_number is not None and current_block_lines:
            results[current_line_number] = "".join(current_block_lines)
            print(f"警告: ファイルの最後に開始ブロック '{current_line_number}' の終了マーカーが見つかりませんでした。", file=sys.stderr)

    except FileNotFoundError:
        print(f"エラー: '{output_file}' が見つかりません。", file=sys.stderr)
        return None

    return results


def eval_parsed_data(parsed_data, plot=True, plot_thre=100):
    success_result_num = 0
    for line_num, content in sorted(parsed_data.items()):
        for line in content.split('\n'):
            if "success" in line:
                success_result_num += 1
    print(f"{success_result_num}個のsuccessデータが見つかりました。")

    for line_num, content in sorted(parsed_data.items()):
        param_line_match = re.search(r"Parameters: (.+)", content)
        if param_line_match:
            print(f"\n--- Input Line {line_num} の結果 ---")
            print("  今回のパラメータ")
            print(f"    {param_line_match.group(1)}")

        output_lines = content.split('\n')
        # "Status: success" の行を探す
        for i, line in enumerate(output_lines):
            if "Status" in line.strip():
                params = {}
                base_convex_hull_points = []
                # 5行上まで遡ってパラメータを抽出
                if i >= 5: # 少なくとも5行上まで存在するか確認
                    for j in range(i - 5, i):
                        current_line = output_lines[j].strip()
                        if current_line.startswith("kin-scale"):
                            params["kin-scale"] = float(current_line.split(" ")[1])
                        elif current_line.startswith("base-size-scale"):
                            params["base-size-scale"] = float(current_line.split(" ")[1])
                        elif current_line.startswith("Base convex hull"):
                            # 凸包の座標を正規表現で抽出
                            match = re.search(r'#f\((.*?)\)', current_line)
                            if match:
                                coords_str = match.group(1)
                                # カンマで区切られた座標ペアを分割し、数値に変換
                                # #f(x y) の形式なので、まず ) で分割し、その後 #f( を取り除く
                                point_strs = re.findall(r'#f\(([^)]*)\)', current_line)
                                for point_str in point_strs:
                                    try:
                                        x, y = map(float, point_str.split())
                                        base_convex_hull_points.append((x, y))
                                    except ValueError:
                                        print(f"Warning: Could not parse point '{point_str}'")
                if params:
                    print("  Status:")
                    if "success" in line:
                        print("    success")
                    else:
                        print("    fail")
                    if base_convex_hull_points:
                        print("  Base convex hullの頂点座標")
                        for i, point in enumerate(base_convex_hull_points):
                            print(f"    Point {i+1}: ({point[0]}, {point[1]})")
                        area_cm2 = calculate_polygon_area(base_convex_hull_points)
                        print(f"  面積")
                        print(f"    {int(area_cm2)}cm^2")
                        if plot and area_cm2 < plot_thre:
                            plot_convex_hull_with_circle(
                                base_convex_hull_points,
                                circle_diameter=80,
                                title=f"Optimized vacuum base shape ({int(area_cm2)}cm^2)"
                                )
                    else:
                        print("Base convex hullの頂点座標は見つかりませんでした。")
                    print("")
                else:
                    print("条件を満たす 'Status: success' の行が見つからないか、必要なパラメータを抽出できませんでした。")


def calculate_polygon_area(vertices):
    """
    Return area [cm^2]
    """
    n = len(vertices)
    if n < 3:
        return 0.0
    area = 0.0
    for i in range(n):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % n]
        area += (x1 * y2 - x2 * y1)
    return abs(area) / 2.0 / 100.0


def plot_convex_hull_with_circle(points, circle_diameter, title):
    """
    凸包の頂点と中心に円を可視化します。

    Args:
        points (list of tuple): 凸包の頂点のリスト (例: [(x1, y1), (x2, y2), ...])。
        circle_diameter_mm (float): 中心に描画する円の直径 (mm)。
        title (str, optional): グラフのタイトル。デフォルトは"Convex Hull with Circle"。
    """

    points = np.array(points)
    # 凸包を閉じるために最初の点を最後に追加
    closed_hull = np.vstack([points, points[0]])
    circle_radius = circle_diameter / 2.0
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.fill(closed_hull[:, 0], closed_hull[:, 1], 'skyblue', alpha=0.3, label='Base Area')
    circle = plt.Circle((0, 0), circle_radius, color='red', fill=False, linestyle='--', label=f'Vacuum Pad')
    ax.add_patch(circle)
    # グラフの設定
    ax.set_aspect('equal', adjustable='box')  # アスペクト比を等しく設定
    min_x, max_x = ax.get_xlim()
    min_y, max_y = ax.get_ylim()
    ax.set_xticks(np.arange(np.floor(min_x / 20) * 20, np.ceil(max_x / 20) * 20 + 1, 20))
    ax.set_yticks(np.arange(np.floor(min_y / 20) * 20, np.ceil(max_y / 20) * 20 + 1, 20))
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.set_xlabel('X [mm]')
    ax.set_ylabel('Y [mm]')
    ax.set_title(title)
    ax.legend(loc='upper left')  # 凡例の場所
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='グリッドサーチの結果をパースして評価します。')
    parser.add_argument(
        '--plot', action='store_true',
        help='結果をグラフで描画する場合に指定します。')
    args = parser.parse_args()

    parsed_data = parse_grid_search_output('output.txt')
    if parsed_data:
        eval_parsed_data(parsed_data, plot=args.plot, plot_thre=100)
    else:
        print("パースするデータがありませんでした。")
