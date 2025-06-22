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

(defclass rigid-leg-contact-constraint
  :super default-contact-constraint
  :slots ()
  )

(defmethod rigid-leg-contact-constraint
  (:init-from-coords
   (support-polygon-coords)
   (let ((f (instance face :init
                      :vertices (list (float-vector -50.0 -50.0 0.0)
                                      (float-vector -50.0 50.0 0.0)
                                      (float-vector 50.0 50.0 0.0)
                                      (float-vector 50.0 -50.0 0.0)
                                      (float-vector -50.0 -50.0 0.0))
                      :normal #f(0 0 -1))))
     (send-super :init-from-support-polygon
           :support-polygon f
           :support-polygon-coords support-polygon-coords
           :mu-trans 0.8
           :mu-rot 0.002)
     ))
  )

;; (defclass rigid-leg-contact-constraint
;;   :super contact-constraint
;;   :slots ()
;;   )

;; (defmethod rigid-leg-contact-constraint
;;   (:init
;;    (&key  (name))
;;    "Calc default constraint matrix.
;;     This is include 2D friction force, 1D rotational friction moment, non-negative normal force, and 2D-COP constraint,
;;     e.g., for foot constraint.
;; "
;;    ;; サポートポリゴンを付けたほうがいいかも？
;;    (setq contact-constraint-list
;;          (append
;;           (list
;;            ;; friction
;;            (instance 2D-translational-friction-contact-constraint :init 0.8)
;;            (instance rotational-friction-contact-constraint :init 0.02 :fz)
;;            ;; cop
;;            ;; (instance 2D-cop-contact-constraint :init 0 0 0 0)
;;            ;; (instance 2D-cop-contact-constraint :init 20 -20 20 -20)
;;            ;; fz
;;            (instance norm-contact-constraint :init :fz)
;;            (instance no-moment-constraint :init)
;;            )))

;;    (send-super :init :name name)
;;    )
;;   )

(defclass vacuum-pad-contact-constraint
  :super contact-constraint
  :slots ()
  )

(defmethod vacuum-pad-contact-constraint
  ;; Mainly copied from util.l
  (:init-from-coords
   (poly-coords
    &key
    (mu-trans 0.1)
    (mu-rot 0.001)
    (suction-force -200) ;; negative pressure force [N]
    (name)
    ((:support-polygon-margin margin) 0) ;; [mm]
    )
   (let* ((poly (instance face :init
                          :vertices (list (float-vector -50.0 -50.0 0.0)
                                          (float-vector -50.0 50.0 0.0)
                                          (float-vector 50.0 50.0 0.0)
                                          (float-vector 50.0 -50.0 0.0)
                                          (float-vector -50.0 -50.0 0.0))
                          :normal #f(0 0 -1)))
          (vs (mapcar
                #'(lambda (v) (send poly-coords :inverse-transform-vector v))
                (send poly :vertices)
                )
              ))
     ;; Mainly copied from default-contact-constraint
     (setq contact-constraint-list
           (list
            ;; friction
            (instance 2D-translational-friction-contact-constraint :init mu-trans)
            (instance rotational-friction-contact-constraint :init mu-rot :fz)
            ;; cop
            (instance 2D-cop-contact-constraint :init
                      (max (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) margin) 0)
                      (min (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) margin) 0)
                      (max (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) margin) 0)
                      (min (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) margin) 0))
            ;; fz
            (instance min-max-contact-constraint :init :fz suction-force :min/max :min)
            )))
   (send-super :init :name name)
   )
  )

;; (defclass vacuum-pad-contact-constraint
;;   :super contact-constraint
;;   :slots ()
;;   )

;; ;; See default-constraint class for reference at eus_qp/euslisp/contact-optimization.l
;; (defmethod vacuum-pad-contact-constraint
;;   (:init
;;    ;; 直径80mmの吸着パッドに-40kPaを書けると、およそ200N
;;    (&key (suction-force -200) (name))
;;    "Calc default constraint matrix.
;;     This is include 2D friction force, 1D rotational friction moment, non-negative normal force, and 2D-COP constraint,
;;     e.g., for foot constraint.
;; "
;;    ;; (setq contact-constraint-list
;;    ;;       (append
;;    ;;        (list
;;    ;;         ;; friction 摩擦力は出せない
;;    ;;         (instance 2D-translational-friction-contact-constraint :init 0)
;;    ;;         (instance rotational-friction-contact-constraint :init 0 :fz)
;;    ;;         ;; 吸着パッドからモーメントは発生しない
;;    ;;         (instance no-moment-constraint :init)
;;    ;;         ;; max-fz 最大吸着力をfpとして、fz+fp>=0、すなわちfz>=-fp=suction-force。符号の向きに注意。
;;    ;;         (instance min-max-contact-constraint :init :fz suction-force :min/max :min)
;;    ;;         (instance min-max-contact-constraint :init :fz -150 :min/max :max)
;;    ;;         )))

;;    ;; サポートポリゴンを付けたほうがいいかも？
;;    (setq contact-constraint-list
;;          (append
;;           (list
;;            ;; friction
;;            (instance 2D-translational-friction-contact-constraint :init 0.8)
;;            (instance rotational-friction-contact-constraint :init 0.02 :fz)
;;            ;; cop
;;            ;; (instance 2D-cop-contact-constraint :init 0 0 0 0)
;;            ;; (instance 2D-cop-contact-constraint :init 20 -20 20 -20)
;;            ;; fz
;;            ;; (instance norm-contact-constraint :init :fz :norm -1)
;;            (instance no-moment-constraint :init)
;;            (instance min-max-contact-constraint :init :fz suction-force :min/max :min)
;;            ;; (instance min-max-contact-constraint :init :fz -50 :min/max :max)
;;            )))

;;    (send-super :init :name name)
;;    )
;;   )
