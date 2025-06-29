(require "yamaguchi_6axis_arm_with_vacuum_base.l")

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
  :super yamaguchi_6axis_arm_with_vacuum_base-robot
  :slots ()
  )

;; TODO: check robot weight (each link weight)
(defmethod yamaguchi-arm-robot
  (:init
   ()
   (send-super :init)
   (send self :add-rigid-legs)
   (send self :add-vacuum-pad)
   (send self :init-ending)
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
       (setq leg (make-cube 30 30 20))
       (send leg :set-color (float-vector 1 1 0.3))
       (send leg :translate (float-vector 0 0 (* 0.5 (z-of-cube leg))) :local)
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
          (height 10)
          vacuum-pad-link vacuum-pad-joint vacuum-pad-coords
          (vacuum-pad (make-cylinder 40 height))
          vacuum-pad-normal-link vacuum-pad-normal-joint vacuum-pad-normal-coords
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
     (send vacuum-pad :set-color #f(0.3 0.3 1))
     (setq vacuum-pad-normal-link (instance bodyset-link
                                     :init (make-cascoords)
                                     :bodies (list vacuum-pad)
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
     )
   nil)
  (:invariant-joint-list
   ()
   (let (end-coords-list invariant-joint-list)
     (dolist (coords-name (list :rigid-leg0-coords :rigid-leg1-coords :rigid-leg2-coords :rigid-leg3-coords :vacuum-pad-coords :vacuum-pad-normal-coords))
       (setq end-coords-list (append end-coords-list (list (send self :get coords-name)))))
     (dolist (ec end-coords-list)
       (let ((target-link (send ec :parent)))
         (while (send target-link :joint)
           (setq invariant-joint-list (append invariant-joint-list (list (send target-link :joint))))
           (setq target-link (send (send target-link :joint) :parent-link)))))
     invariant-joint-list))
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
  ;; By setting leg initial pose, optmotiongen calculation is likely to succeed
  (:leg-init
   (&key (pos 50)) ;; [mm]
   (send (send self :leg-joint 0 :x) :joint-angle 50)
   (send (send self :leg-joint 0 :y) :joint-angle 50)
   (send (send self :leg-joint 1 :x) :joint-angle -50)
   (send (send self :leg-joint 1 :y) :joint-angle 50)
   (send (send self :leg-joint 2 :x) :joint-angle -50)
   (send (send self :leg-joint 2 :y) :joint-angle -50)
   (send (send self :leg-joint 3 :x) :joint-angle 50)
   (send (send self :leg-joint 3 :y) :joint-angle -50)
   )
  (:end-coords
   ()
   (send self :rarm :end-coords))
  (:base-cube
   ()
   (let* ((x-size 160) (y-size 130) (z-size 80)
          (base-cube (make-cube x-size y-size z-size)))
     (send base-cube :newcoords (send self :copy-worldcoords))
     (send base-cube :translate (float-vector 0 0 (* 0.5 z-size)))
     (send base-cube :set-color (float-vector 0 1 0 0.3))
     base-cube))
  (:move-rigid-leg-to-original
   ()
   (let ((av (send self :angle-vector)))
     (setf (elt av 8)  -64.0)
     (setf (elt av 9)  -62.0)
     (setf (elt av 10)  64.0)
     (setf (elt av 11) -62.0)
     (setf (elt av 12) -64.0)
     (setf (elt av 13)  38.0)
     (setf (elt av 14)  64.0)
     (setf (elt av 15)  38.0)
     (send self :angle-vector av))
   )
  (:draw-support-polygon
   (&key (flush t))
   (let ((pad-size 40) pad-pos pad-edges ;; padの半径
         (leg-size 2.5) leg-pos leg-edges) ;; legの一辺の長さの半分
     ;; vacuum-pad
     (setq pad-pos (send (send self :get :vacuum-pad-coords) :worldpos))
     (dotimes (i 8)
       (setq pad-edges
             (append
              pad-edges
              (list
               (instance edge :init
                         :pvertex (v+ (float-vector (* pad-size (cos (* i (/ pi 4))))
                                                    (* pad-size (sin (* i (/ pi 4))))
                                                    -1)
                                      pad-pos)
                         :nvertex (v+ (float-vector (* pad-size (cos (* (+ i 1) (/ pi 4))))
                                                    (* pad-size (sin (* (+ i 1) (/ pi 4))))
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
     (send-all (append pad-edges leg-edges) :draw-on :flush flush :color (float-vector 0.3 1 0.3) :width 3))
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
               :draw-on :flush flush :color (float-vector 1 1 1) :width 1)
     )
   )
  (:draw-suction-arrow
   (&key (flush t))
   (let ((arrow-color (float-vector 1 0.3 0.3))
         (arrow-width 3))
     (with-set-color-line-width
      (send *irtviewer* :viewer) arrow-color arrow-width
      (send (send *irtviewer* :viewer) :draw-arrow
            (float-vector 0 0 0) (float-vector 0 0 -50)) :flush flush)
     ))
  (:set-leg-color
   (&optional (color (float-vector 1 1 0.3)))
   ;; (send leg :set-color color))
   (let (leg-link)
     (dolist (link-name (list "rigid_leg0_link" "rigid_leg1_link" "rigid_leg2_link" "rigid_leg3_link"))
       (setq leg-link (find link-name (send r :links) :test #'(lambda (x y) (equal x (send y :name)))))
       (send-all (send leg-link :bodies) :set-color color)))
   )
  )

(defun yamaguchi-arm ()
  (instance yamaguchi-arm-robot :init)
  )

(defun draw-contact-constraints (robot)
  (let ((r (copy-object robot)))
    (objects (list r))
    (send *irtviewer* :viewer :viewing :look
          #f(273.173 910.441 465.739)
          #f(7.75047 42.2827 80.3897)
          #f(0.0 0.0 1.0))
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :draw-objects)

    (send r :newcoords (make-coords))
    (send r :move-rigid-leg-to-original)
    (send r :set-leg-color (float-vector 0 0 0 0)) ;; transparent
    (send *irtviewer* :draw-origin nil) ;; Remove IRT viewer origin
    (send r :draw-support-polygon :flush nil)
    (send r :draw-friction-cone :flush nil)
    (send r :draw-suction-arrow :flush nil)
    (send *irtviewer* :flush)
    )
  )
