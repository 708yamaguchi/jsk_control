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
   (send self :add-rigid-legs)
   (send self :add-vacuum-pad)
   (send self :init-ending)
   )
  (:add-rigid-legs
   ()
   (let ((base-link (find "base_link" (send self :links)
                          :test #'(lambda (x y) (equal x (send y :name)))))
         leg leg-link leg-joint leg-links leg-joint-list leg-coords leg-coords-list
         (link-names (list "rigid_leg1" "rigid_leg2" "rigid_leg3" "rigid_leg4"))
         (joint-names (list "rigid_leg_joint1" "rigid_leg_joint2" "rigid_leg_joint3" "rigid_leg_joint4"))
         (coords-names (list :rigid-leg1-coords :rigid-leg2-coords :rigid-leg3-coords :rigid-leg4-coords))
         (positions (list (float-vector 50 50 0) (float-vector -50 50 0) (float-vector 50 -50 0) (float-vector -50 -50 0)))
         (min-max (list (list -100000 100000) (list -100000 10000) (list -100000 100000) (list -100000 100000)))
         )
     (dotimes (i (length link-names))
       ;; links
       (setq leg (make-cube 30 30 20))
       (send leg :set-color #f(1 1 0.3))
       (send leg :translate (float-vector 0 0 (* 0.5 (z-of-cube leg))) :local)
       (setq leg-link (instance bodyset-link
                                :init (make-cascoords)
                                :bodies (list leg)
                                :name (elt link-names i)))
       ;; TODO: check robot weight
       ;; (send leg-link :weight 1)
       ;; (send leg-link :inertia-tensor #2f((1 1 1)(1 1 1)(1 1 1)))
       ;; (setq (leg-link . acentroid) (float-vector -2.56219083301764 -36.620809748659596 -0.0050794825177114)) 
       (send leg-link :translate (elt positions i))
       (send base-link :assoc leg-link)
       (setq leg-links (append leg-links (list leg-link)))
       ;; joints
       ;; TODO: 2 dof joint is necessary。ダミーのbodyset-linkが必要か、virtual joint listの方を参考にする
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
     (dotimes (i (length coords-names))
       (setq leg-coords (make-cascoords :coords (send (elt leg-links i) :copy-worldcoords)))
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
     (setq vacuum-pad-coords (make-cascoords :coords (send vacuum-pad-link :copy-worldcoords)))
     (send vacuum-pad-link :assoc vacuum-pad-coords)
     (send self :put :vacuum-pad-coords vacuum-pad-coords)
     )
   nil)
  (:invariant-joint-list
   ()
   ;; TODO: 各limbに対して再帰的にjoint-listを取得できるようにする
   (list (send (send (send self :get :rigid-leg1-coords) :parent) :joint)
         (send (send (send self :get :rigid-leg2-coords) :parent) :joint)
         (send (send (send self :get :rigid-leg3-coords) :parent) :joint)
         (send (send (send self :get :rigid-leg4-coords) :parent) :joint)
         (send (send (send self :get :vacuum-pad-coords) :parent) :joint)
         )
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
  ;; (let ((r (instance yamaguchi_6axis_arm_with_vacuum_base-robot :init)))
  ;;   ;; (send r :add-vacuum-base-coords)
  ;;   ;; (send r :add-vacuum-base-limbs)
  ;;   ;; (send r :init-ending)
  ;;   r))
  (instance yamaguchi-arm-robot :init)
  )
