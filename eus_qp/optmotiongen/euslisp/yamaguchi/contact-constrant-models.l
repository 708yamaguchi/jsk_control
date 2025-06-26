(require "package://eus_qp/euslisp/contact-optimization.l")
(require "package://eus_qp/optmotiongen/euslisp/util.l")

(defclass no-moment-constraint
  :super contact-constraint
  )

(defmethod no-moment-constraint
  (:init
   (&key (name))
   "Calc no-moment constraint matrix.
    Constraint is considered as C w = 0 <=> C w >=0 and C w <= 0."
   (let ((ret (apply
               #'append
               (mapcar #'(lambda (axis)
                           (list (calc-constraint-param-list-for-min-max axis 0 :min/max :min)
                                 (calc-constraint-param-list-for-min-max axis 0 :min/max :max)))
                       '(:nx :ny :nz)))))
     (setq constraint-param-list
           (list :matrix (apply #'append (mapcar #'(lambda (x) (cadr (memq :matrix x))) ret))
                 :vector (apply #'append (mapcar #'(lambda (x) (cadr (memq :vector x))) ret)))))
   (send-super :init :name name))
  )

(defclass no-friction-constraint
  :super contact-constraint
  )

(defmethod no-friction-constraint
  (:init
   (&key (name))
   "Calc no-moment constraint matrix.
    Constraint is considered as C w = 0 <=> C w >=0 and C w <= 0."
   (let ((ret (apply
               #'append
               (mapcar #'(lambda (axis)
                           (list (calc-constraint-param-list-for-min-max axis 0 :min/max :min)
                                 (calc-constraint-param-list-for-min-max axis 0 :min/max :max)))
                       '(:fx :fy)))))
     (setq constraint-param-list
           (list :matrix (apply #'append (mapcar #'(lambda (x) (cadr (memq :matrix x))) ret))
                 :vector (apply #'append (mapcar #'(lambda (x) (cadr (memq :vector x))) ret)))))
   (send-super :init :name name))
  )

(defclass rigid-leg-contact-constraint
  :super contact-constraint
  :slots ()
  )

(defmethod rigid-leg-contact-constraint
  (:init
   (&key (name) (size) (shape :square))
   "Calc default constraint matrix.
    This is include 2D friction force, 1D rotational friction moment, non-negative normal force, and 2D-COP constraint,
    e.g., for foot constraint.
"
   (setq contact-constraint-list
         (append
          (list
           ;; friction
           ;; 静止摩擦係数を下げると、逆に収束しやすくなる（出力されるベースサイズが小さくなる）ことがある。
           (instance 2D-translational-friction-contact-constraint :init 0.3)
           (instance rotational-friction-contact-constraint :init 0.005 :fz)
           ;; moment
           ;; 剛体脚がモーメントを発生させられることにより、吸着パッド全体で大きなモーメントに耐えられる
           (cond
            ((equal shape :square)
             (instance 2D-cop-contact-constraint :init size (* -1 size) size (* -1 size)))
            ;; 半径sizeの円に内接する八角形
            ((equal shape :octagon)
             (let (vertices (size 40) octagon-face)
               (dotimes (i 8)
                 (setq vertices (append vertices (list (scale size (float-vector (cos (* i (/ pi 4)))
                                                                                 (sin (* i (/ pi 4)))
                                                                                 0))))))
               (setq octagon-face (elt (send (make-prism vertices 1) :faces) 0)) ;; normalは逆かも？
               (instance polygon-cop-contact-constraint :init octagon-face))
             )
            )
           ;; fz 垂直抗力は正の方向
           (instance norm-contact-constraint :init :fz)
           )))

   (send-super :init :name name)
   )
  )

(defclass vacuum-pad-contact-constraint
  :super contact-constraint
  :slots ()
  )

;; 吸引力を発生させるだけの接触点
;; 吸着パッドは、rigid-leg-contact-constraint + vacuum-pad-contact-constraintの組み合わせで表現できる。
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
           ;; ;; friction
           (instance no-friction-constraint :init)
           ;; moment
           (instance no-moment-constraint :init)
           ;; fz
           (instance min-max-contact-constraint :init :fz suction-force :min/max :min)
           (instance min-max-contact-constraint :init :fz 0 :min/max :max) ;; Make sure that suction is negative
           )))

   (send-super :init :name name)
   )
  )
