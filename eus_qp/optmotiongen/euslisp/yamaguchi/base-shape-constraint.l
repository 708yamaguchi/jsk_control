;; ベースの形状を長方形にするための線形等式・不等式制約
(defun joint-index-func (name joint-list)
  (position name joint-list :test #'(lambda (x y) (equal x (send y :name)))))

(defun rigid-legs-invariant? (invariant-joint-list)
  (let* ((invariant-leg-index-list
          (mapcar #'(lambda (leg-joint)
                      (joint-index-func leg-joint invariant-joint-list))
                  (list "rigid_leg0_x_joint" "rigid_leg1_x_joint" "rigid_leg2_x_joint" "rigid_leg3_x_joint"
                        "rigid_leg0_y_joint" "rigid_leg1_y_joint" "rigid_leg2_y_joint" "rigid_leg3_y_joint")))
         (invariant? (reduce #'(lambda (x y) (and x y)) invariant-leg-index-list)))
    invariant?))

(defmethod instant-configuration-task
  ;; Add equality constraint for rectangle base shape
  (:invariant-config-equality-constraint-matrix
   (&key (update? nil))
   "
return $\bm{A}_{\mathit{invar}} \in \mathbb{R}^{0 \times \mathit{dim}(\bm{q}_{\mathit{invar}})}$ (no equality constraint)
   "
   (unless (rigid-legs-invariant? _invariant-joint-list)
     ;; default value
     (return-from :invariant-config-equality-constraint-matrix (make-matrix 0 _dim-invariant-config)))
   (let ((m (make-matrix 4 _dim-invariant-config)) vec)
     (setq vec (make-list _dim-invariant-config :initial-element 0))
     (setf (elt vec (eval (joint-index-func "rigid_leg1_x_joint" _invariant-joint-list))) 1)
     (setf (elt vec (eval (joint-index-func "rigid_leg2_x_joint" _invariant-joint-list))) -1)
     (setf (matrix-row m 0) (coerce vec float-vector))
     ;;
     (setq vec (make-list _dim-invariant-config :initial-element 0))
     (setf (elt vec (eval (joint-index-func "rigid_leg3_x_joint" _invariant-joint-list))) 1)
     (setf (elt vec (eval (joint-index-func "rigid_leg0_x_joint" _invariant-joint-list))) -1)
     (setf (matrix-row m 1) (coerce vec float-vector))
     ;;
     (setq vec (make-list _dim-invariant-config :initial-element 0))
     (setf (elt vec (eval (joint-index-func "rigid_leg0_y_joint" _invariant-joint-list))) 1)
     (setf (elt vec (eval (joint-index-func "rigid_leg1_y_joint" _invariant-joint-list))) -1)
     (setf (matrix-row m 2) (coerce vec float-vector))
     ;;
     (setq vec (make-list _dim-invariant-config :initial-element 0))
     (setf (elt vec (eval (joint-index-func "rigid_leg2_y_joint" _invariant-joint-list))) 1)
     (setf (elt vec (eval (joint-index-func "rigid_leg3_y_joint" _invariant-joint-list))) -1)
     (setf (matrix-row m 3) (coerce vec float-vector))
     ))
  ;; Add equality constraint for base shape
  (:invariant-config-equality-constraint-vector
   (&key (update? t))
   "
return $\bm{b}_{\mathit{invar}} \in \mathbb{R}^{0}$ (no equality constraint)
   "
   (unless (rigid-legs-invariant? _invariant-joint-list)
     ;; default value
     (return-from :invariant-config-equality-constraint-vector (float-vector)))
   (float-vector 0 0 0 0))
  ;; 各剛体脚の座標系に大小を付ける。rigid_leg0がX座標Y座標ともに大きくて、rigid_leg2はその反対側の頂点。
  (:phi-inequality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
&& \left\{ \begin{array}{ll}
\bm{\phi}_{\mathit{min}} \leq \bm{\phi} + \Delta \bm{\phi} \leq \bm{\phi}_{\mathit{max}} & \\
- \Delta \bm{\phi}_{\mathit{limit}} \leq \Delta \bm{\phi} \leq \Delta \bm{\phi}_{\mathit{limit}} & ({\rm if} \ \ \Delta \bm{\phi}_{\mathit{limit}} \ \ {\rm is \ set})
\end{array} \right. \\
\Leftrightarrow&&\begin{pmatrix} \bm{I} \\ - \bm{I} \\ \bm{I} \\ - \bm{I}\end{pmatrix} \Delta \bm{\phi}
\geq \begin{pmatrix} \bm{\phi}_{\mathit{min}} - \bm{\phi} \\ - (\bm{\phi}_{\mathit{max}} - \bm{\phi}) \\ - \Delta \bm{\phi}_{\mathit{limit}} \\ - \Delta \bm{\phi}_{\mathit{limit}} \end{pmatrix} \\
\Leftrightarrow&&\bm{C_{\phi}} \Delta \bm{\phi} \geq \bm{d_{\phi}}
\end{eqnarray}

return $\bm{C_{\phi}} := \begin{pmatrix} \bm{I} \\ - \bm{I} \\ \bm{I} \\ - \bm{I}\end{pmatrix} \in \mathbb{R}^{4 N_{\mathit{invar\mathchar`-joint}} \times N_{\mathit{invar\mathchar`-joint}}}$
   "
   (when (or update? (not (send self :get :phi-inequality-constraint-matrix)))
     (send self :put :phi-inequality-constraint-matrix
           (apply
            #'concatenate-matrix-column
            (remove
             nil
             (list
              (unit-matrix _dim-phi)
              (scale-matrix -1 (unit-matrix _dim-phi))
              (if (send self :delta-phi-limit-vector)
                  (unit-matrix _dim-phi))
              (if (send self :delta-phi-limit-vector)
                  (scale-matrix -1 (unit-matrix _dim-phi)))
              )))
           )
     )
   (unless (rigid-legs-invariant? _invariant-joint-list)
     ;; default value
     (return-from :phi-inequality-constraint-matrix (send self :get :phi-inequality-constraint-matrix)))
   ;; Add constraint to original one
   (let* ((orig-matrix (send self :get :phi-inequality-constraint-matrix))
          (new-matrix (make-matrix (+ (send orig-matrix :get-val 'dim0) 2) (send orig-matrix :get-val 'dim1))))
     ;; Copy original constraint matrix
     (dotimes (i (send orig-matrix :get-val 'dim0))
       (setf (matrix-row new-matrix i) (matrix-row orig-matrix i)))
     ;; Add base shape constraint
     ;; TODO
     new-matrix)
   )
  (:phi-inequality-constraint-vector
   (&key (update? t))
   "
return $\bm{d_{\phi}} := \begin{pmatrix} \bm{\phi}_{\mathit{min}} - \bm{\phi} \\ - (\bm{\phi}_{\mathit{max}} - \bm{\phi}) \\ - \Delta \bm{\phi}_{\mathit{limit}} \\ - \Delta \bm{\phi}_{\mathit{limit}} \end{pmatrix} \in \mathbb{R}^{4 N_{\mathit{invar\mathchar`-joint}}}$
   "
   (when (or update? (not (send self :get :phi-inequality-constraint-vector)))
     (send self :put :phi-inequality-constraint-vector
           (apply
            #'concatenate
            float-vector
            (remove
             nil
             (list
              (v- (send self :phi-min-vector) _phi-vector)
              (v- _phi-vector (send self :phi-max-vector))
              (if (send self :delta-phi-limit-vector)
                  (scale -1 (send self :delta-phi-limit-vector)))
              (if (send self :delta-phi-limit-vector)
                  (scale -1 (send self :delta-phi-limit-vector)))
              )))
           )
     )
   (unless (rigid-legs-invariant? _invariant-joint-list)
     ;; default value
     (return-from :phi-inequality-constraint-vector (send self :get :phi-inequality-constraint-vector)))
   ;; Add constraint to original one
   (let* ((orig-vector (send self :get :phi-inequality-constraint-vector))
          (new-vector (coerce (make-list (+ (length orig-vector) 2) :initial-element 0) float-vector)))
     ;; Copy original constraint vector
     (dotimes (i (length orig-vector))
       (setf (elt new-vector i) (elt orig-vector i)))
     ;; Add base shape constraint
     ;; TODO
     new-vector)
   )
)


;; ベースのサイズを小さくするための目的関数の設定
;; (defmethod instant-configuration-task
;;   (:posture-task-value
;;    (&key (update? t))
;;    "
;; ベースのサイズを小さくするための制約
;;    "
;;    (map float-vector
;;         #'(lambda (j a sc)
;;             (* sc (send j :angle-to-speed (- a (send j :joint-angle)))))
;;         _posture-joint-list
;;         _posture-joint-angle-list
;;         _target-posture-scale-list
;;         )
;;    )
;;   (:task-value
;;    (&key (update? t))
;;    "
;; return $\bm{e}(\bm{q})
;; := \begin{pmatrix} \bm{e}^{\mathit{kin}}(\bm{q}) \\ \bm{e}^{\mathit{eom\mathchar`-trans}}(\bm{q}) \\ \bm{e}^{\mathit{eom\mathchar`-rot}}(\bm{q}) \\ \bm{e}^{\mathit{trq}}(\bm{q}) \\ \bm{e}^{\mathit{posture}}(\bm{q}) \end{pmatrix}
;; = \begin{pmatrix} \bm{e}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \\ \bm{e}^{\mathit{eom\mathchar`-trans}}(\bm{\hat{w}}) \\ \bm{e}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}, \bm{\hat{w}}, \bm{\phi}) \\ \bm{e}^{\mathit{trq}}(\bm{\theta}, \bm{\hat{w}}, \bm{\tau}, \bm{\phi}) \\ \bm{e}^{\mathit{posture}}(\bm{\theta}) \end{pmatrix}$
;;    "
;;    (concatenate
;;     float-vector
;;     (send self :kinematics-task-value :update? update?)
;;     (if (not _only-kinematics?) (send self :eom-trans-task-value :update? update?))
;;     (if (not _only-kinematics?) (send self :eom-rot-task-value :update? update?))
;;     (if (and (not _only-kinematics?) (> _num-drive-joint 0))
;;         (send self :torque-task-value :update? update?))
;;     (if (> _num-posture-joint 0)
;;         (send self :posture-task-value :update? update?))
;;     (send self :base-size-task-value :update? update?)
;;     )
;;    )
;; )
