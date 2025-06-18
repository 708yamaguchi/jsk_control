(require "yamaguchi_6axis_arm_with_vacuum_base.l")

(defclass yamaguchi-arm-robot
  :super yamaguchi_6axis_arm_with_vacuum_base-robot
  :slots ()
  )

(defmethod yamaguchi-arm-robot
  (:init
   ()
   (send-super :init)
   ;; (send r :add-vacuum-base-coords)
   (send self :add-vacuum-base-limbs)
   (send self :init-ending)
   )
  (:add-vacuum-base-coords
   ()
   (let ((base-link (find "base_link" (send self :links)
                          :test #'(lambda (x y) (equal x (send y :name)))))
         (rigid-leg-names (list :rigid-leg1-coords :rigid-leg2-coords :rigid-leg3-coords :rigid-leg4-coords))
         (rigid-leg-positions (list (float-vector 50 50 0) (float-vector -50 50 0) (float-vector -50 50 0) (float-vector -50 -50 0))))
     ;; TODO: 剛体脚の位置は直動関節で変えられるようにする
     ;; Define rigid legs fixed to base_link
     (dotimes (i (length rigid-leg-names))
       (send self :put (elt rigid-leg-names i)
             (make-cascoords
              :coords (send (send self :copy-worldcoords) :translate (elt rigid-leg-positions i))
              :parent base-link)))
     ;; Defind vacuum pad fixed to base_link
     (send self :put :vacuum-pad-coords
           (make-cascoords
            :coords (send self :copy-worldcoords)
            :parent base-link))
     )
   )
  (:add-vacuum-base-limbs
   ()
   (let ((base-link (find "base_link" (send self :links)
                          :test #'(lambda (x y) (equal x (send y :name)))))
         leg leg-link leg-joint leg-links leg-joint-list leg-coords leg-coords-list
         (link-names (list "rigid_leg1" "rigid_leg2" "rigid_leg3" "rigid_leg4"))
         (joint-names (list "rigid_leg_joint1" "rigid_leg_joint2" "rigid_leg_joint3" "rigid_leg_joint4"))
         (positions (list (float-vector 50 50 0) (float-vector -50 50 0) (float-vector 50 -50 0) (float-vector -50 -50 0)))
         ;; (min-max (list (list 0 100) (list -100 0) (list 0 100) (list -100 0)))
         (min-max (list (list -100000 100000) (list -100000 10000) (list -100000 100000) (list -100000 100000)))
         )
     (dotimes (i (length link-names))
       ;; links
       (setq leg (make-cube 50 50 50))
       ;; (send leg :translate (float-vector 0 0 (* 0.5 (z-of-cube leg))) :local)
       (setq leg-link (instance bodyset-link
                                :init (make-cascoords)
                                :bodies (list leg)
                                :name (elt link-names i)))
       ;; TODO: check robot weight
       (send leg-link :weight 1)
       (send leg-link :inertia-tensor #2f((1 1 1)(1 1 1)(1 1 1)))
       (setq (leg-link . acentroid) (float-vector -2.56219083301764 -36.620809748659596 -0.0050794825177114)) 
       (send leg-link :translate (elt positions i))
       (send base-link :assoc leg-link)
       (setq leg-links (append leg-links (list leg-link)))
       ;; joints
       ;; TODO: 2 dof joint is necessary
       (setq leg-joint
             (instance linear-joint :init
             ;; (instance rotational-joint :init
                       :name (elt joint-names i)
                       :parent-link base-link :child-link (elt leg-links i)
                       :axis (float-vector 1 0 0)
                       :min (car (elt min-max i)) :max (cadr (elt min-max i))
                       :max-joint-velocity 10.0
                       :max-joint-torque 999999.0))
       (setq leg-joint-list (append leg-joint-list (list leg-joint))))
     ;; Overwrite links and joint-list
     (setq links (append links leg-links))
     (setq joint-list (append joint-list leg-joint-list))
     (setq leg-coords (make-cascoords :coords (send (elt leg-links 0) :copy-worldcoords)))
     (send (elt leg-links 0) :assoc leg-coords)
     (send self :put :rigid-leg1-coords leg-coords)
     (setq leg-coords (make-cascoords :coords (send (elt leg-links 1) :copy-worldcoords)))
     (send (elt leg-links 1) :assoc leg-coords)
     (send self :put :rigid-leg2-coords leg-coords)
     (setq leg-coords (make-cascoords :coords (send (elt leg-links 2) :copy-worldcoords)))
     (send (elt leg-links 2) :assoc leg-coords)
     (send self :put :rigid-leg3-coords leg-coords)
     (setq leg-coords (make-cascoords :coords (send (elt leg-links 3) :copy-worldcoords)))
     (send (elt leg-links 3) :assoc leg-coords)
     (send self :put :rigid-leg4-coords leg-coords)
     ;; TODO: create link for vacuum pad
     (send self :put :vacuum-pad-coords
           (make-cascoords
            :coords (send self :copy-worldcoords)
            :parent base-link))
     )
   nil)
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
  ;; (let ((r (instance yamaguchi_6axis_arm_with_vacuum_base-robot :init)))
  ;;   ;; (send r :add-vacuum-base-coords)
  ;;   ;; (send r :add-vacuum-base-limbs)
  ;;   ;; (send r :init-ending)
  ;;   r))
  (instance yamaguchi-arm-robot :init)
  )
