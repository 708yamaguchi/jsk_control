(require "yamaguchi_6axis_arm_nejineji.l")
(require "yamaguchi_4axis_arm_nejineji.l")
(require "yamaguchi_6axis_arm_nejineji_short.l")
(require "yamaguchi_4axis_arm_nejineji_short.l")
;; (require "yamaguchi_6axis_arm_with_vacuum_base.l")

(require "gl.l")
(defconstant green-color (float-vector 0.3 1.0 0.3))

(defmacro with-set-color-line-width
  (vw color line-width &rest bodies)
  (with-gensyms
   (prev-color prev-width)
   `(let* ((,prev-color (send ,vw :viewsurface :color))
           (,prev-width (send ,vw :viewsurface :line-width)))
      (unwind-protect
          (progn
            (send ,vw :viewsurface :color ,color)
            (send ,vw :viewsurface :line-width ,line-width)
            ,@bodies)
        (send ,vw :viewsurface :color ,prev-color)
        (send ,vw :viewsurface :line-width ,prev-width)
        ))))

(defclass yamaguchi-arm-robot
  ;; :super yamaguchi_6axis_arm_nejineji-robot
  ;; :super yamaguchi_4axis_arm_nejineji-robot
  ;; :super yamaguchi_6axis_arm_nejineji_short-robot
  :super yamaguchi_4axis_arm_nejineji_short-robot
  ;; :super yamaguchi_6axis_arm_with_vacuum_base-robot
  :slots (_pad-diameter _base-cube _wrist-cube _forearm-cube _camera-cube)
  )

(defmethod yamaguchi-arm-robot
  (:init
   (&key (pad-diameter))
   (send-super :init)
   (setq _pad-diameter pad-diameter)
   (send self :add-rigid-legs)
   (send self :add-vacuum-pad)
   (print (send self :descendants))
   ;; (send self :fix-pad-diameter)
   ;; overwrite bodies to return draw-things links not (send link :bodies)
   ;; bodies include rigid legs and vacuum pad
   (setq bodies (flatten (mapcar #'(lambda (b) (if (find-method b :bodies) (send b :bodies))) (send self :all-links))))
   )
  (:add-rigid-legs
   ()
   (let ((base-link (find "base_link" (send self :links)
                          :test #'(lambda (x y) (equal x (send y :name)))))
         leg leg-link leg-joint-x leg-joint-y leg-links leg-joint-list leg-coords
         dummy-link dummy-links
         ;; Rectanble shape  y
         ;; leg1 leg0        |
         ;; leg2 leg3        -> x
         (coords-names (list :rigid-leg0-coords :rigid-leg1-coords :rigid-leg2-coords :rigid-leg3-coords))
         (min-angle -1e5) (max-angle 1e5)
         )
     (dotimes (i (length coords-names))
       ;; base -> x linear joint -> dummy link
       ;; See (:_generate-cascaded-link-with-root-virtual) for reference
       (setq dummy-link (instance bodyset-link
                                  :init (make-cascoords)
                                  :bodies (list (make-cube 1e-3 1e-3 1e-3))
                                  :name (format nil "dummy_rigid_leg~A_link" i)
                                  :weight 0))
       (send base-link :assoc dummy-link)
       (setq dummy-links (append dummy-links (list dummy-link)))
       (setq leg-joint-x
             (instance linear-joint :init
                       :name (format nil "rigid_leg~A_x_joint" i)
                       :parent-link base-link :child-link dummy-link
                       :axis (float-vector 1 0 0)
                       :min min-angle :max max-angle
                       :max-joint-velocity 10.0
                       :max-joint-torque 999999.0))
       (setq leg-joint-list (append leg-joint-list (list leg-joint-x)))
       ;; dummy-link -> y linear joint -> leg link
       (setq leg (make-cube 10 10 1)) ;; モデル的には2.5mm*2.5mmだが、見にくいので大きめに描画
       (send leg :set-color green-color)
       ;; 移動させるとロボットに食い込んで見にくい
       ;; (send leg :translate (float-vector 0 0 (* 0.5 (z-of-cube leg))) :local)
       (setq leg-link (instance bodyset-link
                                :init (make-cascoords)
                                :bodies (list leg)
                                :name (format nil "rigid_leg~A_link" i)
                                :weight 0))
       (send dummy-link :assoc leg-link)
       (setq leg-links (append leg-links (list leg-link)))
       (setq leg-joint-y
             (instance linear-joint :init
                       :name (format nil "rigid_leg~A_y_joint" i)
                       :parent-link dummy-link :child-link leg-link
                       :axis (float-vector 0 1 0)
                       :min min-angle :max max-angle
                       :max-joint-velocity 10.0
                       :max-joint-torque 999999.0))
       (setq leg-joint-list (append leg-joint-list (list leg-joint-y))))
     (dolist (j leg-joint-list)
       (send (send j :child-link) :add-joint j)
       (send (send j :child-link) :add-parent-link (send j :parent-link))
       (send (send j :parent-link) :add-child-links (send j :child-link)))

     ;; Overwrite links and joint-list
     (setq links (append links dummy-links leg-links))
     (setq joint-list (append joint-list leg-joint-list))
     (dotimes (i (length coords-names))
       (setq leg-coords (make-cascoords
                         :coords (send (elt leg-links i) :copy-worldcoords)
                         :name (format nil ":rigid-leg~A-coords" i)))
       (send (elt leg-links i) :assoc leg-coords)
       (send self :put (elt coords-names i) leg-coords)))
   nil)
  (:add-vacuum-pad
   ()
   (let* ((base-link (find "base_link" (send self :links)
                           :test #'(lambda (x y) (equal x (send y :name)))))
          vacuum-pad-link vacuum-pad-joint vacuum-pad-coords
          vacuum-pad-normal-link vacuum-pad-normal-joint vacuum-pad-normal-coords
          ;; vacuum-pad-radius-link vacuum-pad-radius-joint vacuum-pad-radius-coords
          )
     ;; 吸引力発生用
     (setq vacuum-pad-link (instance bodyset-link
                                     :init (make-cascoords)
                                     :bodies (list (make-cube 1e-3 1e-3 1e-3))
                                     :name "vacuum_pad"))
     (send base-link :assoc vacuum-pad-link)
     (setq vacuum-pad-joint
           (instance rotational-joint :init
                     :name "vacuum_pad_joint"
                     :parent-link base-link :child-link vacuum-pad-link
                     :axis (float-vector 0 0 1)
                     :min 0 :max 0
                     :max-joint-velocity 0
                     :max-joint-torque 0))
     (setq links (append links (list vacuum-pad-link)))
     (setq joint-list (append joint-list (list vacuum-pad-joint)))
     (setq vacuum-pad-coords (make-cascoords
                              :coords (send vacuum-pad-link :copy-worldcoords)
                              :name ":vacuum-pad-coords"))
     (send vacuum-pad-link :assoc vacuum-pad-coords)
     (send self :put :vacuum-pad-coords vacuum-pad-coords)
     ;; 垂直抗力用
     (setq vacuum-pad-normal-link (instance bodyset-link
                                     :init (make-cascoords)
                                     :bodies (list (make-cube 1e-3 1e-3 1e-3))
                                     :name "vacuum_pad_normal"))
     (send base-link :assoc vacuum-pad-normal-link)
     (setq vacuum-pad-normal-joint
           (instance rotational-joint :init
                     :name "vacuum_pad_normal_joint"
                     :parent-link base-link :child-link vacuum-pad-normal-link
                     :axis (float-vector 0 0 1)
                     :min 0 :max 0
                     :max-joint-velocity 0
                     :max-joint-torque 0))
     (setq links (append links (list vacuum-pad-normal-link)))
     (setq joint-list (append joint-list (list vacuum-pad-normal-joint)))
     (setq vacuum-pad-normal-coords (make-cascoords
                              :coords (send vacuum-pad-normal-link :copy-worldcoords)
                              :name ":vacuum-pad-normal-coords"))
     (send vacuum-pad-normal-link :assoc vacuum-pad-normal-coords)
     (send self :put :vacuum-pad-normal-coords vacuum-pad-normal-coords)
     (dolist (j (list vacuum-pad-joint vacuum-pad-normal-joint))
       (send (send j :child-link) :add-joint j)
       (send (send j :child-link) :add-parent-link (send j :parent-link))
       (send (send j :parent-link) :add-child-links (send j :child-link)))
     ;; ;; vacuum-pad-radius-jointの長さが、可変サイズ吸着パッドの半径になる。
     ;; (setq vacuum-pad-radius-link (instance bodyset-link
     ;;                                        :init (make-cascoords)
     ;;                                        :bodies (list (make-cube 1e-3 1e-3 1e-3))
     ;;                                        :name "vacuum_pad_radius"))
     ;; (send base-link :assoc vacuum-pad-radius-link)
     ;; (setq vacuum-pad-radius-joint
     ;;       (instance linear-joint :init
     ;;                 :name "vacuum_pad_radius_joint"
     ;;                 :parent-link base-link :child-link vacuum-pad-radius-link
     ;;                 :axis (float-vector 0 0 1)
     ;;                 :min 10 :max 100 ;; 吸着パッドのサイズを0にはしない
     ;;                 :max-joint-velocity 0
     ;;                 :max-joint-torque 0))
     ;; (send vacuum-pad-radius-joint :joint-angle (* 0.5 _pad-diameter))
     ;; (setq links (append links (list vacuum-pad-radius-link)))
     ;; (setq joint-list (append joint-list (list vacuum-pad-radius-joint)))
     ;; (setq vacuum-pad-radius-coords (make-cascoords
     ;;                                 :coords (send vacuum-pad-radius-link :copy-worldcoords)
     ;;                                 :name ":vacuum-pad-radius-coords"))
     ;; (send vacuum-pad-radius-link :assoc vacuum-pad-radius-coords)
     ;; (send self :put :vacuum-pad-radius-coords vacuum-pad-radius-coords)
     ;; (dolist (j (list vacuum-pad-radius-joint))
     ;;   (send (send j :child-link) :add-joint j)
     ;;   (send (send j :child-link) :add-parent-link (send j :parent-link))
     ;;   (send (send j :parent-link) :add-child-links (send j :child-link)))
     )
   nil)
  (:invariant-joint-list
   ()
   (let (end-coords-list invariant-joint-list)
     (dolist (coords-name (list :rigid-leg0-coords :rigid-leg1-coords :rigid-leg2-coords :rigid-leg3-coords :vacuum-pad-coords :vacuum-pad-normal-coords
                                ;; :vacuum-pad-radius-coords
                                ))
       (setq end-coords-list (append end-coords-list (list (send self :get coords-name)))))
     (dolist (ec end-coords-list)
       (let ((target-link (send ec :parent)))
         (while (send target-link :joint)
           (setq invariant-joint-list (append invariant-joint-list (list (send target-link :joint))))
           (setq target-link (send (send target-link :joint) :parent-link)))))
     invariant-joint-list))
  (:variant-joint-list
   ()
   (set-difference (send self :joint-list) (send self :invariant-joint-list)))
  (:leg-joint
   (index axis) ;; axis is :x or :y
   (let* ((coords-name
           (elt (list :rigid-leg0-coords :rigid-leg1-coords :rigid-leg2-coords :rigid-leg3-coords) index))
          (leg-link (send (send self :get coords-name) :parent))
          (leg-y-joint (send leg-link :joint))
          (leg-x-joint (send (send leg-y-joint :parent-link) :joint)))
     (when (eq axis :x)
       (return-from :leg-joint leg-x-joint))
     (when (eq axis :y)
       (return-from :leg-joint leg-y-joint))
     )
   nil)
  (:pad-diameter
   (&optional (diameter))
   (let ((j (find "vacuum_pad_radius_joint"
                  (send self :joint-list)
                  :test #'(lambda (x y) (equal x (send y :name))))))
     (if j ;; if "vacuum_pad_radius_joint" is defined
         (if diameter
             (send j :joint-angle (* 0.5 diameter))
           (* 2 (send j :joint-angle)))
       (if diameter
           (setq _pad-diameter diameter)
         _pad-diameter)))
   )
  (:fix-pad-diameter
   ()
   (let ((j (find "vacuum_pad_radius_joint"
                  (send self :joint-list)
                  :test #'(lambda (x y) (equal x (send y :name))))))
     (send j :min-angle (send j :joint-angle))
     (send j :max-angle (send j :joint-angle)))
   )
  (:unfix-pad-diameter
   ()
   (let ((j (find "vacuum_pad_radius_joint"
                  (send self :joint-list)
                  :test #'(lambda (x y) (equal x (send y :name))))))
     (send j :min-angle 10) ;; 吸着パッドのサイズを0にはしない
     (send j :max-angle 100))
   )
  ;; By setting leg initial pose, optmotiongen calculation is likely to succeed
  (:leg-init
   (&key (pos 50)) ;; [mm]
   (send (send self :leg-joint 0 :x) :joint-angle pos)
   (send (send self :leg-joint 0 :y) :joint-angle pos)
   (send (send self :leg-joint 1 :x) :joint-angle (* -1 pos))
   (send (send self :leg-joint 1 :y) :joint-angle pos)
   (send (send self :leg-joint 2 :x) :joint-angle (* -1 pos))
   (send (send self :leg-joint 2 :y) :joint-angle (* -1 pos))
   (send (send self :leg-joint 3 :x) :joint-angle pos)
   (send (send self :leg-joint 3 :y) :joint-angle (* -1 pos))
   )
  (:end-coords
   ()
   (send self :rarm :end-coords))
  ;; アームとベースの間のcollision判定を高速化するための物体
  ;; 干渉計算が増えるだけでなく、jointが増えるので、最適化は遅くなる。
  (:fix-gripper
   ()
   (let ((gripper-joint1 (find "gripper_joint1"
                               (send self :joint-list)
                               :test #'(lambda (x y)
                                         (substringp x (send y :name)))))
         (gripper-joint2 (find "gripper_joint2"
                               (send self :joint-list)
                               :test #'(lambda (x y)
                                         (substringp x (send y :name))))))
     (dolist (j (list gripper-joint1 gripper-joint2))
       (send j :min-angle (send j :joint-angle))
       (send j :max-angle (send j :joint-angle)))))
  (:unfix-gripper
   ()
   (let ((gripper-joint1 (find "gripper_joint1"
                               (send self :joint-list)
                               :test #'(lambda (x y)
                                         (substringp x (send y :name)))))
         (gripper-joint2 (find "gripper_joint2"
                               (send self :joint-list)
                               :test #'(lambda (x y)
                                         (substringp x (send y :name))))))
     (dolist (j (list gripper-joint1 gripper-joint2))
       (send j :min-angle -15)
       (send j :max-angle 15))))
  (:base-cube
   ()
   (unless _base-cube
     (let* ((base-link (find "base_link" (send self :links)
                           :test #'(lambda (x y) (equal x (send y :name)))))
            (x-size 150) (y-size 125) (z-size 80)
            (base-cube (make-cube x-size y-size z-size)))
       (send base-cube :newcoords (send self :copy-worldcoords))
       (send base-cube :translate (float-vector 0 -12.5 (* 0.5 z-size)))
       (send base-cube :set-color (float-vector 0 1 0 0)) ;; transparent
       (send self :assoc base-cube)
       (setq _base-cube (instance bodyset-link
                                  :init (make-cascoords)
                                  :bodies (list base-cube)
                                  :name "base_cube"))
       (send base-link :assoc _base-cube)
       (setq base-cube-joint
             (instance rotational-joint :init
                       :name "base_cube_joint"
                       :parent-link base-link :child-link _base-cube
                       :axis (float-vector 0 0 1)
                       :min 0 :max 0
                       :max-joint-velocity 0
                       :max-joint-torque 0))
     (setq links (append links (list _base-cube)))
     (setq joint-list (append joint-list (list base-cube-joint)))
     (dolist (j (list base-cube-joint))
       (send (send j :child-link) :add-joint j)
       (send (send j :child-link) :add-parent-link (send j :parent-link))
       (send (send j :parent-link) :add-child-links (send j :child-link)))))
   _base-cube)
  (:wrist-cube
   ()
   (unless _wrist-cube
     ;; Only yamaguchi_6axis_arm_nejineji-robot is supported
     (let* ((wrist-link (find "module4_link1" (send self :links)
                              :test #'(lambda (x y) (equal x (send y :name)))))
            (x-size 70) (y-size 40) (z-size 30)
            (wrist-cube (make-cube x-size y-size z-size)))
       (send wrist-cube :newcoords (send wrist-link :copy-worldcoords))
       (send wrist-cube :translate (float-vector 15 0 (- (* 0.5 z-size) 5)))
       (send wrist-cube :set-color (float-vector 0 1 0 0)) ;; transparent
       (send self :assoc wrist-cube)
       (setq _wrist-cube (instance bodyset-link
                                  :init (make-cascoords)
                                  :bodies (list wrist-cube)
                                  :name "wrist_cube"))
       (send wrist-link :assoc _wrist-cube)
       (setq wrist-cube-joint
             (instance rotational-joint :init
                       :name "wrist_cube_joint"
                       :parent-link wrist-link :child-link _wrist-cube
                       :axis (float-vector 0 0 1)
                       :min 0 :max 0
                       :max-joint-velocity 0
                       :max-joint-torque 0))
     (setq links (append links (list _wrist-cube)))
     (setq joint-list (append joint-list (list wrist-cube-joint)))
     (dolist (j (list wrist-cube-joint))
       (send (send j :child-link) :add-joint j)
       (send (send j :child-link) :add-parent-link (send j :parent-link))
       (send (send j :parent-link) :add-child-links (send j :child-link)))))
   _wrist-cube)
  (:forearm-cube
   ()
   (unless _forearm-cube
     ;; Only yamaguchi_6axis_arm_nejineji-robot is supported
     (let* ((arm-link (find "module2_link1" (send self :links)
                              :test #'(lambda (x y) (equal x (send y :name)))))
            (x-size 40) (y-size 65) (z-size 160)
            (forearm-cube (make-cube x-size y-size z-size)))
       (send forearm-cube :newcoords (send arm-link :copy-worldcoords))
       (send forearm-cube :translate (float-vector 0 0 110))
       (send forearm-cube :set-color (float-vector 0 1 0 0)) ;; transparent
       (send self :assoc forearm-cube)
       (setq _forearm-cube (instance bodyset-link
                                  :init (make-cascoords)
                                  :bodies (list forearm-cube)
                                  :name "forearm_cube"))
       (send arm-link :assoc _forearm-cube)
       (setq forearm-cube-joint
             (instance rotational-joint :init
                       :name "forearm_cube_joint"
                       :parent-link arm-link :child-link _forearm-cube
                       :axis (float-vector 0 0 1)
                       :min 0 :max 0
                       :max-joint-velocity 0
                       :max-joint-torque 0))
     (setq links (append links (list _forearm-cube)))
     (setq joint-list (append joint-list (list forearm-cube-joint)))
     (dolist (j (list forearm-cube-joint))
       (send (send j :child-link) :add-joint j)
       (send (send j :child-link) :add-parent-link (send j :parent-link))
       (send (send j :parent-link) :add-child-links (send j :child-link)))))
   _forearm-cube)
  (:camera-cube
   ()
   (unless _camera-cube
     ;; Only yamaguchi_6axis_arm_nejineji-robot is supported
     (let* ((arm-link (find "module4_link2" (send self :links)
                              :test #'(lambda (x y) (equal x (send y :name)))))
            (x-size 60) (y-size 25) (z-size 80)
            (camera-cube (make-cube x-size y-size z-size)))
       (send camera-cube :newcoords (send arm-link :copy-worldcoords))
       (send camera-cube :translate (float-vector 0 55 -28))
       (send camera-cube :set-color (float-vector 0 1 0 0)) ;; transparent
       (send self :assoc camera-cube)
       (setq _camera-cube (instance bodyset-link
                                  :init (make-cascoords)
                                  :bodies (list camera-cube)
                                  :name "camera_cube"))
       (send arm-link :assoc _camera-cube)
       (setq camera-cube-joint
             (instance rotational-joint :init
                       :name "camera_cube_joint"
                       :parent-link arm-link :child-link _camera-cube
                       :axis (float-vector 0 0 1)
                       :min 0 :max 0
                       :max-joint-velocity 0
                       :max-joint-torque 0))
     (setq links (append links (list _camera-cube)))
     (setq joint-list (append joint-list (list camera-cube-joint)))
     (dolist (j (list camera-cube-joint))
       (send (send j :child-link) :add-joint j)
       (send (send j :child-link) :add-parent-link (send j :parent-link))
       (send (send j :parent-link) :add-child-links (send j :child-link)))))
   _camera-cube)
  (:move-rigid-leg-to-original
   ()
     (send (send self :leg-joint 0 :x) :joint-angle -64.0)
     (send (send self :leg-joint 0 :y) :joint-angle -62.0)
     (send (send self :leg-joint 1 :x) :joint-angle  64.0)
     (send (send self :leg-joint 1 :y) :joint-angle -62.0)
     (send (send self :leg-joint 2 :x) :joint-angle -64.0)
     (send (send self :leg-joint 2 :y) :joint-angle  38.0)
     (send (send self :leg-joint 3 :x) :joint-angle  64.0)
     (send (send self :leg-joint 3 :y) :joint-angle  38.0)
   )
  (:fix-leg
   ()
   (dolist (index (list 0 1 2 3))
     (dolist (axis (list :x :y))
       (let ((j (send self :leg-joint index axis)))
         (send j :min-angle (send j :joint-angle))
         (send j :max-angle (send j :joint-angle)))))
   )
  (:unfix-leg
   ()
   (dolist (index (list 0 1 2 3))
     (dolist (axis (list :x :y))
       (send (send self :leg-joint index axis) :min-angle -1e5)
       (send (send self :leg-joint index axis) :max-angle 1e5)
       ))
   )
  (:draw-support-polygon
   (&key (flush t))
   (let (pad-pos pad-edges
         (leg-size 2.5) leg-pos leg-edges) ;; legの一辺の長さの半分
     ;; vacuum-pad
     (setq pad-pos (send (send self :get :vacuum-pad-coords) :worldpos))
     (dotimes (i 8)
       (setq pad-edges
             (append
              pad-edges
              (list
               (instance edge :init
                         :pvertex (v+ (float-vector (* (* 0.5 (send self :pad-diameter)) (cos (* i (/ pi 4))))
                                                    (* (* 0.5 (send self :pad-diameter)) (sin (* i (/ pi 4))))
                                                    -1)
                                      pad-pos)
                         :nvertex (v+ (float-vector (* (* 0.5 (send self :pad-diameter)) (cos (* (+ i 1) (/ pi 4))))
                                                    (* (* 0.5 (send self :pad-diameter)) (sin (* (+ i 1) (/ pi 4))))
                                                    -1)
                                      pad-pos))))))
     ;; rigid legs
     (dolist (limb-coords (list :rigid-leg0-coords :rigid-leg1-coords :rigid-leg2-coords :rigid-leg3-coords))
       (setq leg-pos (send (send self :get limb-coords) :worldpos))
       (setq leg-vertices nil)
       (dotimes (i 4)
         (setq leg-edges
               (append
                leg-edges
                (list
                 (instance edge :init
                           :pvertex (v+ (float-vector (* (sqrt 2) leg-size (cos (* (+ i 0.5) (/ pi 2))))
                                                      (* (sqrt 2) leg-size (sin (* (+ i 0.5) (/ pi 2))))
                                                      -1)
                                        leg-pos)
                           :nvertex (v+ (float-vector (* (sqrt 2) leg-size (cos (* (+ i 1.5) (/ pi 2))))
                                                      (* (sqrt 2) leg-size (sin (* (+ i 1.5) (/ pi 2))))
                                                      -1)
                                        leg-pos)
                           ))))))
     ;; Visualize
     (send-all (append pad-edges leg-edges) :draw-on :flush flush :color green-color :width 5))
   )
  (:draw-friction-cone
   (&key (flush t))
   (let ((cone (make-cone (float-vector 0 0 0)
                          (list (float-vector 9 9 30) (float-vector 9 -9 30) (float-vector -9 -9 30) (float-vector -9 9 30))))
         pad-pos pad-edges leg-pos leg-edges)
     ;; vacuum-pad
     (setq pad-pos (send (send self :get :vacuum-pad-coords) :worldpos))
     (setq cone (copy-object cone))
     (send cone :locate pad-pos :world)
     (send cone :worldcoords)
     (setq pad-edges (send cone :edges))
     ;; leg
     (dolist (limb-coords (list :rigid-leg0-coords :rigid-leg1-coords :rigid-leg2-coords :rigid-leg3-coords))
       (setq cone (copy-object cone))
       (setq leg-pos (send (send self :get limb-coords) :worldpos))
       (send cone :locate leg-pos :world)
       (send cone :worldcoords)
       (send cone :edges)
       (setq leg-edges (append leg-edges (send cone :edges))))
     ;; visualize
     (send-all (append pad-edges leg-edges)
               :draw-on :flush flush :color (float-vector 1 1 1) :width 5)
     )
   )
  (:draw-suction-arrow
   (&key (flush t))
   (let ((arrow-color (float-vector 1 0.3 0.3))
         (arrow-width 5))
     (with-set-color-line-width
      (send *irtviewer* :viewer) arrow-color arrow-width
      (send (send *irtviewer* :viewer) :draw-arrow
            (float-vector 0 0 0) (float-vector 0 0 -50)) :flush flush)
     ))
  (:set-leg-color
   (&optional (color green-color))
   ;; (send leg :set-color color))
   (let (leg-link)
     (dolist (link-name (list "rigid_leg0_link" "rigid_leg1_link" "rigid_leg2_link" "rigid_leg3_link"))
       (setq leg-link (find link-name (send self :links) :test #'(lambda (x y) (equal x (send y :name)))))
       (send-all (send leg-link :bodies) :set-color color)))
   )
  (:set-pad-color
   (&optional (color green-color))
   (let ((pad-link (find "vacuum_pad" (send self :links) :test #'(lambda (x y) (equal x (send y :name))))))
     (send (send pad-link :bodies) :set-color color))
   )
  (:base-convex-hull
   ()
   ;; 最初はロボット相対で座標を記述
   (let* ((leg-points
           (list (float-vector (send (send self :leg-joint 0 :x) :joint-angle)
                               (send (send self :leg-joint 0 :y) :joint-angle))
                 (float-vector (send (send self :leg-joint 1 :x) :joint-angle)
                               (send (send self :leg-joint 1 :y) :joint-angle))
                 (float-vector (send (send self :leg-joint 2 :x) :joint-angle)
                               (send (send self :leg-joint 2 :y) :joint-angle))
                 (float-vector (send (send self :leg-joint 3 :x) :joint-angle)
                               (send (send self :leg-joint 3 :y) :joint-angle))))
          (pad-points (mapcar #'(lambda (i)
                                  (scale (* 0.5 (send self :pad-diameter)) (float-vector (cos (* i (/ pi 4))) (sin (* i (/ pi 4))))))
                              (list 0 1 2 3 4 5 6 7)))
          (points (append leg-points pad-points)) ;; 点が3つ以上あるのが前提
          lower-hull upper-hull ccw-hull)
     ;; destructive
     ;; X座標を優先的して昇順にソート。X座標が同じ場合は、Y座標で昇順にソート。
     (sort points #'(lambda (p0 p1)
                      (if (eq (elt p0 0) (elt p1 0))
                          (<= (elt p0 1) (elt p1 1))
                        (<= (elt p0 0) (elt p1 0)))))
     ;; 下部凸包を構築
     (dolist (p points)
       (while (and (>= (length lower-hull) 2)
                   (<= (cross-product (elt lower-hull (- (length lower-hull) 2)) (elt lower-hull (- (length lower-hull) 1)) p)
                       0))
         (setq lower-hull (butlast lower-hull)))
       (setq lower-hull (append lower-hull (list p))))
     ;; 上部凸包を構築
     (dolist (p (reverse points)) ;; not destructive
       (while (and (>= (length upper-hull) 2)
                   (<= (cross-product (elt upper-hull (- (length upper-hull) 2)) (elt upper-hull (- (length upper-hull) 1)) p)
                       0))
         (setq upper-hull (butlast upper-hull)))
       (setq upper-hull (append upper-hull (list p))))
     ;; 上部と下部の凸包を結合して、半時計回りのリストを得る。始点と終点が重複するため、除去
     (setq ccw-hull (append (butlast lower-hull) (butlast upper-hull)))
     ;; 座標を3次元にして、ロボット座標相対に姿勢変換
     (mapcar #'(lambda (p)
                 (send (send self :copy-worldcoords)
                       :transform-vector
                       (float-vector (elt p 0) (elt p 1) 0)))
             ccw-hull)
     ))
  (:draw-base-convex-hull
   (&key (flush t))
   (let* ((hull-points (send self :base-convex-hull))
          (hull-length (length hull-points))
          hull-edges)
     (dotimes (i hull-length)
       (setq hull-edges
             (append
              hull-edges
              (list
               (instance edge :init
                         :pvertex (elt hull-points (mod i hull-length))
                         :nvertex (elt hull-points (mod (+ i 1) hull-length)))))))
     ;; Visualize
     (send-all hull-edges :draw-on :flush flush :color (float-vector 0.5 1.0 0.5) :width 5))
   )
  (:base-convex-hull-area
   ()
   (let* ((hull-points (send self :base-convex-hull))  ;; 3点以上あることを前提とする
          (area 0.0)
          (len (length hull-points))
          p1 p2)
     (dotimes (i len)
       (setq p1 (elt hull-points (mod i len)))
       (setq p2 (elt hull-points (mod (+ i 1) len)))
       (setq area (+ area
                     (- (* (elt p1 0) (elt p2 1))
                        (* (elt p2 0) (elt p1 1))))))
     (* (abs area) 0.5)))
  (:random-angle-vector
   ()
   ;; arm joints
   (dolist (j (send self :variant-joint-list))
     (let* ((max-angle (send j :max-angle))
            (min-angle (send j :min-angle))
            (range (float (- max-angle min-angle))))
       (send j :joint-angle (+ (random range) min-angle))))
   ;; rigid leg joints (random value betweewn -200 and 200)
   (dolist (j (send self :invariant-joint-list))
     (send j :joint-angle (- (random 400.0) 200.0)))
   (send self :angle-vector)
   )
  )

(defun set-collada-body-transparency (c-body transparency)
  (unless (derivedp c-body collada-body)
    (return-from set-collada-body-transparency nil))
  (let ((glvertices (send c-body :glvertices)))
    ;; :GL-DISPLAYLIST-IDが残っていると、後から透明度を変更できない
    (gl::delete-displaylist-id (get glvertices :GL-DISPLAYLIST-ID))
    (setf (get glvertices :GL-DISPLAYLIST-ID) nil)
    (setf (get glvertices :transparent) transparency)))

(defun yamaguchi-arm
    (&key (pad-diameter 60)
          (transparency))
  (let ((r (instance yamaguchi-arm-robot
                     :init
                     :pad-diameter pad-diameter))
        (opaque-link-name "base_link") ;; not transparent link name
        target-link
        )
    (when transparency
      (dolist (l (send r :all-links))
        (when (not (equal opaque-link-name (send l :name)))
          (mapcar #'(lambda (c-body)
                      (set-collada-body-transparency c-body transparency))
                  (send l :bodies))))
      )
    (send r :base-cube) ;; for collision avoidance
    (when (derivedp r yamaguchi_6axis_arm_nejineji-robot)
      (send r :wrist-cube) ;; for collision avoidance
      (send r :forearm-cube) ;; for collision avoidance
      (send r :camera-cube) ;; for collision avoidance
      )
    r)
  )

;; (:init関数のうちコメントアウトすると、変な緑の物体が映らない
;; (send self :add-rigid-legs)
;; (send self :add-vacuum-pad)
(defun draw-6axis-arm (robot)
  (let ((r (copy-object robot)))
    (send r :angle-vector
          #f(0.0 0.0 0.0 0.0 -90.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0))
    (objects (list r))
    (send *irtviewer* :viewer :viewing :look
          #f(1213.81 2584.02 2120.5)
          #f(0.0 -19.0 325.807)
          #f(0.0 0.0 1.0))
    (send *irtviewer* :change-background
          (float-vector (/ 192 255.0) (/ 192 255.0) (/ 192 255.0)))
    (send *irtviewer* :draw-origin nil) ;; Remove IRT viewer origin
    (send *irtviewer* :draw-objects)
    )
  )

(defun draw-contact-constraints (robot)
  (let ((r (copy-object robot)))
    (objects (list r))
    ;; Darken light
    (change-scale-light gl::*light0* 0)
    (change-scale-light gl::*light1* 0.6)
    (change-scale-light gl::*light2* 0)
    (change-scale-light gl::*light3* 0)
    (send *irtviewer* :viewer :viewing :look
          #f(273.173 910.441 465.739)
          #f(7.75047 42.2827 80.3897)
          #f(0.0 0.0 1.0))
    (send *irtviewer* :change-background
          (float-vector (/ 192 255.0) (/ 192 255.0) (/ 192 255.0)))
    (send *irtviewer* :draw-origin nil) ;; Remove IRT viewer origin
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :draw-objects)

    (send r :newcoords (make-coords))
    (send r :move-rigid-leg-to-original)
    (send r :set-leg-color (float-vector 0 0 0 0)) ;; transparent
    (send r :draw-support-polygon :flush nil)
    (send r :draw-friction-cone :flush nil)
    (send r :draw-suction-arrow :flush nil)
    (send *irtviewer* :flush)
    )
  ;; Restore light
  (change-scale-light gl::*light0* 1.00)
  (change-scale-light gl::*light1* 0.70)
  (change-scale-light gl::*light2* 0.40)
  (change-scale-light gl::*light3* 0.25)
  )

(defun draw-base-from-below (robot)
  (let* ((r (copy-object robot))
         (vacuum-pad-normal-link
          (find "vacuum_pad_normal" (send r :links)
                :test #'(lambda (x y) (equal x (send y :name)))))
         (vacuum-pad (make-cylinder (* (send r :pad-diameter) 0.5) 10)))
    (send r :newcoords (make-coords))
    ;; Assume yamaguchi_6axis_arm_nejineji
    (send r :angle-vector
          (float-vector 0.0 0.0 0.0 0.0 -90.0 0.0 0.0 0.0
                 -67.0 -67.0 67.0 -67.0 -67.0 43.0 67.0 43.0
                 0.0 0.0 0.0 0.0 0.0 0.0))
    (send vacuum-pad :newcoords (send vacuum-pad-normal-link :copy-worldcoords))
    (send vacuum-pad :translate (float-vector 0 0 -2) :local)
    (send vacuum-pad :set-color green-color)
    (objects (list r vacuum-pad))
    (send *irtviewer* :viewer :viewing :look
          #f(0 0 -1000)
          #f(0.0 0.0 0.0)
          #f(0.0 1.0 0.0))
    (send *irtviewer* :change-background
          (float-vector (/ 192 255.0) (/ 192 255.0) (/ 192 255.0)))
    (send *irtviewer* :draw-origin nil) ;; Remove IRT viewer origin
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :flush)
    )
  )

(defun draw-base-convex-hull (robot &key (v))
  (let* ((r (copy-object robot))
         (vacuum-pad-normal-link
          (find "vacuum_pad_normal" (send r :links)
                :test #'(lambda (x y) (equal x (send y :name)))))
         (vacuum-pad (make-cylinder (* (send r :pad-diameter) 0.5) 10)))
    ;; Set robot pose
    (send r :newcoords (make-coords))
    (send vacuum-pad :newcoords (send vacuum-pad-normal-link :copy-worldcoords))
    (send vacuum-pad :set-color #f(0.3 0.3 1 0)) ;; transparent
    ;; Visualize
    (unless v
      (setq v (send-lexpr (instantiate x::irtviewer) :create nil)))
    (send v :objects (list r vacuum-pad))
    (send v :viewer :viewing :look
          #f(0 0 -1800)
          #f(0.0 0.0 0.0)
          #f(0.0 1.0 0.0))
    (send v :draw-origin nil) ;; Remove IRT viewer origin
    (send v :draw-objects)
    (send v :draw-objects)
    (send r :draw-base-convex-hull :flush nil)
    (send v :flush)
    )
  )

;; 3点 p1, p2, p3 の向きを調べるヘルパー関数
;; > 0: 左回り (反時計回り)
;; < 0: 右回り (時計回り)
;; = 0: 同一直線上
(defun cross-product
    (p1 p2 p3)
   (- (* (- (elt p2 0) (elt p1 0)) (- (elt p3 1) (elt p1 1)))
      (* (- (elt p2 1) (elt p1 1)) (- (elt p3 0) (elt p1 0))))
  )
