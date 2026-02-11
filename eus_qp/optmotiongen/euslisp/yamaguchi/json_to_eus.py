import json
import sys
import os

def convert_json_to_eus(input_path):
    # ファイルの存在確認
    if not os.path.exists(input_path):
        print(f"Error: File not found: {input_path}")
        return

    # 出力ファイル名の生成 (例: motion.json -> motion.l)
    base_name = os.path.splitext(input_path)[0]
    output_path = f"{base_name}.l"

    try:
        # JSONファイルの読み込み
        with open(input_path, 'r') as f:
            data = json.load(f)

        eus_commands = [";; Generated from JSON motion data"]
        
        # 各時刻のデータを処理
        for entry in data.get("motion", []):
            tm = entry.get("time")
            js = entry.get("joint_states", {})
            
            # joint1からjoint6までの値をリスト化
            joints = [js.get(f"joint{i}", 0.0) for i in range(1, 7)]
            
            # EusLispの各行を作成
            tm_str = f"(setq tm {tm})"
            av_str = f"(setq av (float-vector {' '.join(map(str, joints))} 0 0))"
            send_str = "(send robot :angle-vector av)"
            format_str = '(format t "~A: ~A~%" tm (send robot :centroid))'
            
            # 1行にまとめて追加
            eus_commands.append(f"{tm_str} {av_str} {send_str} {format_str}")

        # ファイルへの書き出し
        with open(output_path, 'w') as f:
            f.write("\n".join(eus_commands) + "\n")

        # ユーザへの通知
        print(f"Success! EusLisp file generated at: {output_path}")

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    # 引数のチェック
    if len(sys.argv) < 2:
        print("Usage: python json_to_eus.py <input_json_file>")
    else:
        convert_json_to_eus(sys.argv[1])
