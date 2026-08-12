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
         (min-angle -1) (max-angle 1))
     (setq base-coords-list nil)
     (dotimes (i (length offsets))
       (let* ((offset (elt offsets i))
              (ox (elt offset 0))
              (oy (elt offset 1))
              (init-coords (send base-link :copy-worldcoords))
              ;; 1. base_link -> dummy_link (x-axis linear joint)
              ;; オフセットをかけずに base-link の座標で作成
              (dummy-link (instance bodyset-link
                                    :init (make-cascoords :coords (send init-coords :copy-worldcoords))
                                    :bodies (list (make-cube 1e-3 1e-3 1e-3))
                                    :name (format nil "dummy_base_coords_~A_link" i)
                                    :weight 0))
              (joint-x (instance linear-joint :init
                                 :name (format nil "base_coords_~A_x_joint" i)
                                 :parent-link base-link :child-link dummy-link
                                 :axis (float-vector 1 0 0)
                                 :min (+ ox min-angle) :max (+ ox max-angle)
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
                                 :min (+ oy min-angle) :max (+ oy max-angle)
                                 :max-joint-velocity 10.0
                                 :max-joint-torque 999999.0))
              (bc (make-cascoords :name (format nil "base_coords_~A" i)
                                  :coords (send init-coords :copy-worldcoords))))
         ;; 先に assoc 階層を構成
         (send base-link :assoc dummy-link)
         (send dummy-link :assoc bc-link)
         (send bc-link :assoc bc)
         ;; ジョイント角度にオフセットを設定 (assoc階層に従って動く)
         (send joint-x :joint-angle ox)
         (send joint-y :joint-angle oy)
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

  ;; base-coords のワールド位置リストを返すメソッド
  (:base-coords-pos-list
   ()
   (mapcar #'(lambda (bc) (send bc :worldpos)) base-coords-list))

  ;; ベース原点から四隅までの直動関節 (x, y) の角度 (mm) リストを返すメソッド
  (:base-joint-angles
   ()
   (let (res)
     (dotimes (i (length base-coords-list))
       (let ((jx (send self :joint (format nil "base_coords_~A_x_joint" i)))
             (jy (send self :joint (format nil "base_coords_~A_y_joint" i))))
         (push (list (if jx (send jx :joint-angle) 0.0)
                     (if jy (send jy :joint-angle) 0.0))
               res)))
     (nreverse res)))

  ;; 四隅の位置および2つの直動関節角度を表示するメソッド
  (:print-base-info
   ()
   (format t "===== R8 Base Coords & Joint Angles =====~%")
   (format t "Base Link Worldpos: ~A~%" (send (find "base_link" (send self :links) :test #'(lambda (x y) (equal x (send y :name)))) :worldpos))
   (dotimes (i (length base-coords-list))
     (let* ((bc (elt base-coords-list i))
            (jx (send self :joint (format nil "base_coords_~A_x_joint" i)))
            (jy (send self :joint (format nil "base_coords_~A_y_joint" i))))
       (format t "Corner ~A (~A):~%" i (send bc :name))
       (format t "  Worldpos    : ~A~%" (send bc :worldpos))
       (format t "  Joint X (mm): ~A~%" (if jx (send jx :joint-angle) 0.0))
       (format t "  Joint Y (mm): ~A~%" (if jy (send jy :joint-angle) 0.0))))
   t)

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
   target-arm)

  ;; Python reset_pose の EusLisp 移植
  (:reset-pose
   ()
   ;; Left arm initial pose
   (send self :l_zaxis_joint :joint-angle 700.0)  ;; Middle of range (0.7m -> 700mm)
   (send self :l_shoulder_y_joint :joint-angle (rad2deg 0.0))
   (send self :l_elbow_p1_joint :joint-angle (rad2deg 0.785))  ;; 45 degrees
   (send self :l_elbow_p2_joint :joint-angle (rad2deg 0.785))  ;; 45 degrees
   (send self :l_upper_arm_y_joint :joint-angle (rad2deg 0.0))
   (send self :l_wrist_p_joint :joint-angle (rad2deg 0.0))
   (send self :l_wrist_r_joint :joint-angle (rad2deg 0.0))

   ;; Right arm initial pose
   (send self :r_zaxis_joint :joint-angle 700.0)  ;; Middle of range (0.7m -> 700mm)
   (send self :r_shoulder_y_joint :joint-angle (rad2deg 0.0))
   (send self :r_elbow_p1_joint :joint-angle (rad2deg 0.785))  ;; 45 degrees
   (send self :r_elbow_p2_joint :joint-angle (rad2deg 0.785))  ;; 45 degrees
   (send self :r_upper_arm_y_joint :joint-angle (rad2deg 0.0))
   (send self :r_wrist_p_joint :joint-angle (rad2deg 0.0))
   (send self :r_wrist_r_joint :joint-angle (rad2deg 0.0))

   ;; Elevator initial pose
   (send self :elv_zaxis_joint :joint-angle 700.0)  ;; Middle of range (0.7m -> 700mm)
   (send self :elv_storage_y_joint :joint-angle (rad2deg 0.0))

   (send self :angle-vector))

  ;; Python startup_pose の EusLisp 移植
  (:startup-pose
   ()
   ;; Left arm initial pose
   (send self :l_zaxis_joint :joint-angle 820.0)  ;; 0.82m -> 820mm
   (send self :l_shoulder_y_joint :joint-angle (rad2deg 0.12531464))
   (send self :l_elbow_p1_joint :joint-angle (rad2deg 0.02666055))
   (send self :l_elbow_p2_joint :joint-angle (rad2deg 0.02666055))
   (send self :l_upper_arm_y_joint :joint-angle (rad2deg 0.0))
   (send self :l_wrist_p_joint :joint-angle (rad2deg 0.0))
   (send self :l_wrist_r_joint :joint-angle (rad2deg 0.0))

   ;; Right arm initial pose
   (send self :r_zaxis_joint :joint-angle 820.0)  ;; 0.82m -> 820mm
   (send self :r_shoulder_y_joint :joint-angle (rad2deg -0.12531464))
   (send self :r_elbow_p1_joint :joint-angle (rad2deg 0.02666055))
   (send self :r_elbow_p2_joint :joint-angle (rad2deg 0.02666055))
   (send self :r_upper_arm_y_joint :joint-angle (rad2deg 0.0))
   (send self :r_wrist_p_joint :joint-angle (rad2deg 0.0))
   (send self :r_wrist_r_joint :joint-angle (rad2deg 0.0))

   ;; Elevator initial pose
   (send self :elv_zaxis_joint :joint-angle 421.8)  ;; 0.4218m -> 421.8mm
   (send self :elv_storage_y_joint :joint-angle (rad2deg 0.0))

   (send self :angle-vector))

  ;; Python tuck_arm_pose の EusLisp 移植
  (:tuck-arm-pose
   ()
   ;; Left arm initial pose
   (send self :l_zaxis_joint :joint-angle 650.0)  ;; 0.65m -> 650mm
   (send self :l_shoulder_y_joint :joint-angle (rad2deg 0.124791))
   (send self :l_elbow_p1_joint :joint-angle (rad2deg 0.026369))
   (send self :l_elbow_p2_joint :joint-angle (rad2deg 0.026369))
   (send self :l_upper_arm_y_joint :joint-angle (rad2deg -1.570000))
   (send self :l_wrist_p_joint :joint-angle (rad2deg 0.0))
   (send self :l_wrist_r_joint :joint-angle (rad2deg 0.0))

   ;; Right arm initial pose
   (send self :r_zaxis_joint :joint-angle 950.0)  ;; 0.95m -> 950mm
   (send self :r_shoulder_y_joint :joint-angle (rad2deg -0.124791))
   (send self :r_elbow_p1_joint :joint-angle (rad2deg 0.026661))
   (send self :r_elbow_p2_joint :joint-angle (rad2deg 0.026661))
   (send self :r_upper_arm_y_joint :joint-angle (rad2deg 1.570000))
   (send self :r_wrist_p_joint :joint-angle (rad2deg 0.0))
   (send self :r_wrist_r_joint :joint-angle (rad2deg 0.0))

   ;; Elevator initial pose
   (send self :elv_zaxis_joint :joint-angle 421.8)  ;; 0.4218m -> 421.8mm
   (send self :elv_storage_y_joint :joint-angle (rad2deg 0.0))

   (send self :angle-vector)))

(provide :r8-arm)
