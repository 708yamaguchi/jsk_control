;;
;; r8-arm.l - R8 Robot setup for task optimization
;;

(load "attachable-plane.l")

(unless (boundp '*r8-loaded*)
  (load "r8.l")
  (setq *r8-loaded* t))

(defclass r8-arm-robot
  :super r8-robot
  :slots (target-arm base-coords-list))

(defmethod r8-arm-robot
  ;; インスタンス化時に :arm を指定できるようにする（デフォルトは :rarm）
  (:init
   (&rest args &key ((:arm a) :rarm) &allow-other-keys)
   (setq target-arm a)
   (send-super* :init :name "r8" args)
   
   ;; base-coords-list (cascoords のリスト) を作成し、ジョイントを定義して base_link (bodyset-link) に assoc する
   ;; ロボット原点からx方向に±345, y方向に±235の4点
   (let ((base-link (find "base_link" (send self :links) :test #'(lambda (x y) (equal x (send y :name)))))
         (offsets (list (float-vector  345  235 0)
                        (float-vector -345  235 0)
                        (float-vector -345 -235 0)
                        (float-vector  345 -235 0)))
         dummy-links base-coords-links base-joint-list
         (min-angle -1e5) (max-angle 1e5))
     (setq base-coords-list nil)
     (dotimes (i (length offsets))
       (let* ((offset (elt offsets i))
              (init-coords (send (send base-link :copy-worldcoords) :translate offset :local))
              ;; 1. base_link -> dummy_link (x-axis linear joint)
              (dummy-link (instance bodyset-link
                                    :init (make-cascoords :coords (send init-coords :copy-worldcoords))
                                    :bodies (list (make-cube 1e-3 1e-3 1e-3))
                                    :name (format nil "dummy_base_coords_~A_link" i)
                                    :weight 0))
              (joint-x (instance linear-joint :init
                                 :name (format nil "base_coords_~A_x_joint" i)
                                 :parent-link base-link :child-link dummy-link
                                 :axis (float-vector 1 0 0)
                                 :min min-angle :max max-angle
                                 :max-joint-velocity 10.0
                                 :max-joint-torque 999999.0))
              ;; 2. dummy_link -> base_coords_link (y-axis linear joint)
              (bc-body (make-cube 10 10 1))
              (bc-link (instance bodyset-link
                                 :init (make-cascoords :coords (send init-coords :copy-worldcoords))
                                 :bodies (list bc-body)
                                 :name (format nil "base_coords_~A_link" i)
                                 :weight 0))
              (joint-y (instance linear-joint :init
                                 :name (format nil "base_coords_~A_y_joint" i)
                                 :parent-link dummy-link :child-link bc-link
                                 :axis (float-vector 0 1 0)
                                 :min min-angle :max max-angle
                                 :max-joint-velocity 10.0
                                 :max-joint-torque 999999.0))
              (bc (make-cascoords :name (format nil "base_coords_~A" i)
                                  :coords (send bc-link :copy-worldcoords))))
         (send base-link :assoc dummy-link)
         (send dummy-link :assoc bc-link)
         (send bc-link :assoc bc)
         (setq dummy-links (append dummy-links (list dummy-link)))
         (setq base-coords-links (append base-coords-links (list bc-link)))
         (setq base-joint-list (append base-joint-list (list joint-x joint-y)))
         (setq base-coords-list (append base-coords-list (list bc)))))

     (dolist (j base-joint-list)
       (send (send j :child-link) :add-joint j)
       (send (send j :child-link) :add-parent-link (send j :parent-link))
       (send (send j :parent-link) :add-child-links (send j :child-link)))

     (setq links (append links dummy-links base-coords-links))
     (setq joint-list (append joint-list base-joint-list))
     (setq bodies (flatten (mapcar #'(lambda (b) (if (find-method b :bodies) (send b :bodies))) (send self :all-links)))))
   self)

  ;; base-coords-list 座標系のリストを返すメソッド
  (:base-coords-list
   ()
   base-coords-list)

  ;; タスク共通の不変ジョイント (ベース座標ジョイント + 操作していない側のアームのジョイント)
  (:invariant-joint-list
   ()
   (let (invariant-joint-list)
     (dolist (bc base-coords-list)
       (let ((target-link (send bc :parent)))
         (while (send target-link :joint)
           (setq invariant-joint-list (append invariant-joint-list (list (send target-link :joint))))
           (setq target-link (send (send target-link :joint) :parent-link)))))
     (let ((other-arm (if (eq target-arm :rarm) :larm :rarm)))
       (when (find-method self other-arm)
         (dolist (j (send self other-arm :joint-list))
           (setq invariant-joint-list (append invariant-joint-list (list j))))))
     (remove-duplicates invariant-joint-list)))

  ;; 可変ジョイント (関節の全リスト - 不変ジョイント)
  (:variant-joint-list
   ()
   (set-difference (send self :joint-list) (send self :invariant-joint-list)))

  ;; 指定されているアームの :end-coords を返す
  (:end-coords
   ()
   (send self target-arm :end-coords))

  ;; 後から使用するアームを変更・参照したい場合のためのアクセサメソッド
  (:target-arm
   (&optional a)
   (if a (setq target-arm a))
   target-arm))

(provide :r8-arm)