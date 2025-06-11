(load "models/room610-scene.l")
(setq *room610* (room610))
;; (objects *room610*)

;; 1. 610モデルの中でアームを配置できる平面（長方形）を指定する。例えばXY平面内で、Z軸正方向にロボットアームを置くなら、以下のようにする。angle-listの順番に注意。ZX平面なら、(z, x)の順に与える
;; 最小値、最大値の
;; (setq min-angle-list (list x_min y_min))
;; (setq max-angle-list (list x_max y_max))
;; (setq direction :z+)

;; 2. 1.の情報から、610モデルに配置可能平面をvisualizeする。

;; 3. タスクのターゲット位置を指定する（室岡さんと同じように、arrowで与える）

;; 4. 3.の情報から、610モデルにタスク位置をvisualizeする。

;; 5. 1.のdirection情報から、*robot*のベースを回転させる。

;; 6. 1.と3.の情報から、optmotiongenの制約を作り、実際にベース移動IKを解く。

;; -> これができれば、610の好きな場所を配置可能面/タスク位置にして、ベース移動IKを解けるようになる。
