(load "../inverse-kinematics-wrapper.l")
(load "../discrete-kinematics-configuration-task.l")
(load "../sqp-msc-optimization.l")
(require "models/arrow-object.l")

(load "models/room610-scene.l")

(load "yamaguchi_6axis_arm_with_vacuum_base.l")
(defmethod yamaguchi_6axis_arm_with_vacuum_base-robot
  (:end-coords ()
    (send self :rarm :end-coords)))


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
  (:offset
   ()
   _offset)
  (:min-angle-list
   ()
   _min-angle-list)
  (:max-angle-list
   ()
   _max-angle-list)
  (:normal
   ()
   _normal)
  )


(defun main
    (&key
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  ;; 1. Prepare task environment
  ;; If you change task configuration, change only this section.
  (setq *room610* (room610))
  (setq *attachable-plane* (instance attachable-plane :init
                                     :offset 900
                                     :min-angle-list (list 7750 -250)
                                     :max-angle-list (list 8350 600)
                                     :normal :z+))
  (setq *robot* (instance yamaguchi_6axis_arm_with_vacuum_base-robot :init))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 8000 800 1000) :rpy (list (deg2rad 60) 0 0)))
  (objects (append (send *room610* :objects)
                   (list (send *attachable-plane* :object)
                         *robot*
                         *arrow*)))

  ;; 2. Rotate *robot* along *attachable-plane*
  ;; It is assumed that *robot* and *room610* coords are the same at first.
  (let ((normal (send *attachable-plane* :normal))
        (offset (send *attachable-plane* :offset)))
    (when (eq normal :x+)
      (send *robot* :rotate pi/2 :y)
      (send *robot* :locate (float-vector offset 0 0) :world))
    (when (eq normal :x-)
      (send *robot* :rotate -pi/2 :y)
      (send *robot* :locate (float-vector offset 0 0) :world))
    (when (eq normal :y+)
      (send *robot* :rotate -pi/2 :x)
      (send *robot* :locate (float-vector 0 offset 0) :world))
    (when (eq normal :y-)
      (send *robot* :rotate pi/2 :x)
      (send *robot* :locate (float-vector 0 offset 0) :world))
    (when (eq normal :z+)
      (send *robot* :locate (float-vector 0 0 offset) :world))
    (when (eq normal :z-)
      (send *robot* :rotate pi :x)
      (send *robot* :locate (float-vector 0 0 offset) :world)))

  ;; optimization
  (send *robot*
        :inverse-kinematics-optmotiongen
        (send *arrow* :copy-worldcoords)
        :move-target (send *robot* :end-coords)
        :translation-axis t
        :rotation-axis nil
        :debug-view t
        :root-virtual-mode
        (let ((normal (send *attachable-plane* :normal)))
          (when (or (eq normal :x+) (eq normal :x-)) :planar-x)
          (when (or (eq normal :y+) (eq normal :y-)) :planar-y)
          (when (or (eq normal :z+) (eq normal :z-)) :planar-z))
        ;; The last added element is rotation joint on the plane (continuous rotation)
        :root-virtual-joint-min-angle-list
        (append (send *attachable-plane* :min-angle-list) (list -1e10))
        :root-virtual-joint-max-angle-list
        (append (send *attachable-plane* :max-angle-list) (list 1e10))
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  nil)

;; 5. 1.のnormal情報から、*robot*のベースを回転させる。

;; 6. 1.と3.の情報から、optmotiongenの制約を作り、実際にベース移動IKを解く。

;; -> これができれば、610の好きな場所を配置可能面/タスク位置にして、ベース移動IKを解けるようになる。
