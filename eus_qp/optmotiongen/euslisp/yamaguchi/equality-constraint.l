(defmethod instant-configuration-task
  ;; Add equality constraint for base shape
  (:invariant-config-equality-constraint-matrix
   (&key (update? nil))
   "
return $\bm{A}_{\mathit{invar}} \in \mathbb{R}^{0 \times \mathit{dim}(\bm{q}_{\mathit{invar}})}$ (no equality constraint)
   "
   (let ((joint-index-func #'(lambda (name joint-list)
                          (position name joint-list :test #'(lambda (x y) (equal x (send y :name))))))
         (m (make-matrix 4 _dim-invariant-config))
         vec)
     (setq vec (make-list _dim-invariant-config :initial-element 0))
     (setf (elt vec (eval (funcall joint-index-func "rigid_leg1_x_joint" _invariant-joint-list))) 1)
     (setf (elt vec (eval (funcall joint-index-func "rigid_leg2_x_joint" _invariant-joint-list))) -1)
     (setf (matrix-row m 0) (coerce vec float-vector))
     ;;
     (setq vec (make-list _dim-invariant-config :initial-element 0))
     (setf (elt vec (eval (funcall joint-index-func "rigid_leg3_x_joint" _invariant-joint-list))) 1)
     (setf (elt vec (eval (funcall joint-index-func "rigid_leg0_x_joint" _invariant-joint-list))) -1)
     (setf (matrix-row m 1) (coerce vec float-vector))
     ;;
     (setq vec (make-list _dim-invariant-config :initial-element 0))
     (setf (elt vec (eval (funcall joint-index-func "rigid_leg0_y_joint" _invariant-joint-list))) 1)
     (setf (elt vec (eval (funcall joint-index-func "rigid_leg1_y_joint" _invariant-joint-list))) -1)
     (setf (matrix-row m 2) (coerce vec float-vector))
     ;;
     (setq vec (make-list _dim-invariant-config :initial-element 0))
     (setf (elt vec (eval (funcall joint-index-func "rigid_leg2_y_joint" _invariant-joint-list))) 1)
     (setf (elt vec (eval (funcall joint-index-func "rigid_leg3_y_joint" _invariant-joint-list))) -1)
     (setf (matrix-row m 3) (coerce vec float-vector))
     ))
  ;; Add equality constraint for base shape
  (:invariant-config-equality-constraint-vector
   (&key (update? t))
   "
return $\bm{b}_{\mathit{invar}} \in \mathbb{R}^{0}$ (no equality constraint)
   "
   (float-vector 0 0 0 0)
   )
)
