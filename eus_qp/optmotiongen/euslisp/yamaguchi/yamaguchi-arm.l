(require "yamaguchi_6axis_arm_with_vacuum_base.l")

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
       (send leg :set-color #f(1 1 0.3))
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
          (vacuum-pad (make-cylinder 40 height))
          vacuum-pad-link vacuum-pad-joint vacuum-pad-coords
          )
     (send vacuum-pad :set-color #f(0.3 0.3 1))
     (setq vacuum-pad-link (instance bodyset-link
                                     :init (make-cascoords)
                                     :bodies (list vacuum-pad)
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
     )
   nil)
  (:invariant-joint-list
   ()
   (let (end-coords-list invariant-joint-list)
     (dolist (coords-name (list :rigid-leg0-coords :rigid-leg1-coords :rigid-leg2-coords :rigid-leg3-coords :vacuum-pad-coords))
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
  )

(defun yamaguchi-arm ()
  (instance yamaguchi-arm-robot :init)
  )
