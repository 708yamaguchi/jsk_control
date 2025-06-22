(require "package://eus_qp/euslisp/contact-optimization.l")

(defclass vacuum-pad-contact-constraint
  :super contact-constraint
  :slots ()
  )

;; See default-constraint class for reference at eus_qp/euslisp/contact-optimization.l
(defmethod vacuum-pad-contact-constraint
  (:init
   ;; 直径80mmの吸着パッドに-40kPaを書けると、およそ200N
   (&key (suction-force -200) (name))
   "Calc default constraint matrix.
    This is include 2D friction force, 1D rotational friction moment, non-negative normal force, and 2D-COP constraint,
    e.g., for foot constraint.
"
   (setq contact-constraint-list
         (append
          (list
           ;; friction 摩擦力は出せない
           (instance 2D-translational-friction-contact-constraint :init 0)
           (instance rotational-friction-contact-constraint :init 0 :fz)
           ;; cop (Contact at a single point)
           ;; (instance 2D-cop-contact-constraint :init 0 0 0 0)
           ;; fz 吸着パッドが発生する力は常に負という制約（下の制約の方が正確なので、下だけでいいかも）
           ;; (instance norm-contact-constraint :init :fz :norm -1)
           ;; max-fz 最大吸着力をfpとして、fz+fp>=0、すなわちfz>=-fp=suction-force。符号の向きに注意。
           (instance min-max-contact-constraint :init :fz suction-force :min/max :min)
           )))
   (send-super :init :name name)
   )
  )
