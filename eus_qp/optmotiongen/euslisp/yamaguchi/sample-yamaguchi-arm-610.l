(load "models/room610-scene.l")

(defclass attachable-plane
  :super propertied-object
  :slots (_min-angle-list _max-angle-list _offset _normal)
  :documentation
  "
610モデルの中でアームを配置できる平面（長方形）を指定する。例えばXY平面内で、Z軸正方向にロボットアームを置くなら、以下のようにする。angle-listの順番に注意。ZX平面なら、(z, x)の順に与える
offsetは平面の位置
最小値、最大値、平面の位置は、ワールド座標で指定する。
normalは:x-, :x+, :y-, :y+, :z-, :z+
"
  )

(defmethod attachable-plane
  (:init
   (&key
    (offset)
    (min-angle-list)
    (max-angle-list)
    (normal)
    )
   (setq _offset offset)
   (setq _min-angle-list min-angle-list)
   (setq _max-angle-list max-angle-list)
   (setq _normal normal)
   )
  (:object
   ()
   (let ((mid (mapcar #'(lambda (x y) (* (+ x y) 0.5)) _min-angle-list _max-angle-list))
         (diff (mapcar #'- _max-angle-list  _min-angle-list))
         size pos cube)
     (when (or (eq _normal :z-) (eq _normal :z+))
       (setq size (list (elt diff 0) (elt diff 1) 1))
       (setq pos (list (elt mid 0) (elt mid 1) _offset)))
     (when (or (eq _normal :y-) (eq _normal :y+))
       (setq size (list (elt diff 1) 1 (elt diff 0)))
       (setq pos (list (elt mid 1) _offset (elt mid 0))))
     (when (or (eq _normal :x-) (eq _normal :x+))
       (setq size (list 1 (elt diff 0) (elt diff 1)))
       (setq pos (list _offset (elt mid 0) (elt mid 1))))
     (setq cube (make-cube (elt size 0) (elt size 1) (elt size 2)))
     (send cube :newcoords
           (make-coords :pos (float-vector (elt pos 0) (elt pos 1) (elt pos 2))))
     (send cube :set-color (float-vector 1 0 0 0.5))
     cube)
   )
  )

(defun main ()
  (let ((room610 (room610))
        (p (instance attachable-plane :init
                     :offset 900
                     :min-angle-list (list 7750 -250)
                     :max-angle-list (list 8350 600)
                     :normal :z+)))
    (objects (append (send room610 :objects) (list (send p :object))))
    )
  nil)


;; 3. タスクのターゲット位置を指定する（室岡さんと同じように、arrowで与える）

;; 4. 3.の情報から、610モデルにタスク位置をvisualizeする。

;; 5. 1.のnormal情報から、*robot*のベースを回転させる。

;; 6. 1.と3.の情報から、optmotiongenの制約を作り、実際にベース移動IKを解く。

;; -> これができれば、610の好きな場所を配置可能面/タスク位置にして、ベース移動IKを解けるようになる。
