(require "package://eus_qp/euslisp/contact-optimization.l")


(defclass rigid-leg-contact-constraint
  :super contact-constraint
  :slots ()
  )

;; See default-constraint class for reference at eus_qp/euslisp/contact-optimization.l
(defmethod rigid-leg-contact-constraint
  (:init 
   (&key (mu-margin-ratio 1.0) (mu-trans 0.1) (mu-rot 0.001) (name)) ;; mu-trans and mu-rot is the same as optmotiongen/euslisp/util.l
   "Calc default constraint matrix.
    This is include 2D friction force, 1D rotational friction moment, non-negative normal force, and 2D-COP constraint,
    e.g., for foot constraint.
    mu-margin-ratio is in [0.0, 1.0] and used for reduce contact constraint boudaries.
    If xx-margin-ratio is 1.0, contact constraint boudaries are same as original one.
    If xx-margin-ratio is in (0.0, 1.0), contact constraint boudaries are reduced.
    If xx-margin-ratio is 0.0, no friction or point contact."
   (setq contact-constraint-list
         (append
          (list
           ;; friction
           (instance 2D-translational-friction-contact-constraint :init (* mu-margin-ratio mu-trans))
           (instance rotational-friction-contact-constraint :init (* mu-margin-ratio mu-rot) :fz)
           ;; cop (Contact at a single point)
           (instance 2D-cop-contact-constraint :init 0 0 0 0)
           ;; fz
           (instance norm-contact-constraint :init :fz) ;; TODO change norm-axis and norm(1/-1)
           )))
   (send-super :init :name name)
   )
  )

(defclass vacuum-pad-contact-constraint
  :super contact-constraint
  :slots ()
  )

;; See default-constraint class for reference at eus_qp/euslisp/contact-optimization.l
(defmethod vacuum-pad-contact-constraint
  (:init 
   (&key (mu-margin-ratio 1.0) (mu-trans 0.0) (mu-rot 0.0) (name)) ;; mu-trans and mu-rot is the same as optmotiongen/euslisp/util.l
   "Calc default constraint matrix.
    This is include 2D friction force, 1D rotational friction moment, non-negative normal force, and 2D-COP constraint,
    e.g., for foot constraint.
    mu-margin-ratio is in [0.0, 1.0] and used for reduce contact constraint boudaries.
    If xx-margin-ratio is 1.0, contact constraint boudaries are same as original one.
    If xx-margin-ratio is in (0.0, 1.0), contact constraint boudaries are reduced.
    If xx-margin-ratio is 0.0, no friction or point contact."
   (setq contact-constraint-list
         (append
          (list
           ;; friction
           (instance 2D-translational-friction-contact-constraint :init (* mu-margin-ratio mu-trans))
           (instance rotational-friction-contact-constraint :init (* mu-margin-ratio mu-rot) :fz)
           ;; cop (Contact at a single point)
           (instance 2D-cop-contact-constraint :init 0 0 0 0)
           ;; fz
           (instance norm-contact-constraint :init :fz :norm -1) ;; negative force
           ;; max-fz TODO：符号の向きに注意。最大吸着力も、まともな値を入れる。
           (instance min-max-contact-constraint :init :fz -9999 :min/max :min)
           )))
   (send-super :init :name name)
   )
  )

(defclass hand-contact-constraint
  :super contact-constraint
  :slots ()
  )

;; See default-constraint class for reference at eus_qp/euslisp/contact-optimization.l
(defmethod hand-contact-constraint
  (:init 
   (&key (torque) (name)) ;; torque is like (list x_force y_force z_force)
   "Calc default constraint matrix.
    This is include 2D friction force, 1D rotational friction moment, non-negative normal force, and 2D-COP constraint,
    e.g., for foot constraint."
   (setq contact-constraint-list
         (append
          (list
           ;; cop (Contact at a single point)
           (instance 2D-cop-contact-constraint :init 0 0 0 0)
           ;; max-fz TODO：符号の向きに注意。最大吸着力も、まともな値を入れる。
           (instance min-max-contact-constraint :init :fx (* 0.9 (elt torque 0)) :min/max :min)
           (instance min-max-contact-constraint :init :fx (* 1.1 (elt torque 0)) :min/max :max)
           (instance min-max-contact-constraint :init :fy (* 0.9 (elt torque 1)) :min/max :min)
           (instance min-max-contact-constraint :init :fy (* 1.1 (elt torque 1)) :min/max :max)
           (instance min-max-contact-constraint :init :fz (* 0.9 (elt torque 0)) :min/max :min)
           (instance min-max-contact-constraint :init :fz (* 1.1 (elt torque 0)) :min/max :max)
           )))
   (send-super :init :name name)
   )
  )
