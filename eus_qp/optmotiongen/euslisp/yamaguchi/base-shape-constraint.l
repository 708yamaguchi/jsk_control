;; ベースの形状を長方形にするための線形等式・不等式制約
(defun joint-index-func (name joint-list)
  (position name joint-list :test #'(lambda (x y) (equal x (send y :name)))))

(defun find-joint (name joint-list)
  (find-if #'(lambda (x) (equal name (send x :name))) joint-list))

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
     ;; X1 - X2 = 0
     (setq vec (make-list _dim-invariant-config :initial-element 0))
     (setf (elt vec (eval (joint-index-func "rigid_leg1_x_joint" _invariant-joint-list))) 1)
     (setf (elt vec (eval (joint-index-func "rigid_leg2_x_joint" _invariant-joint-list))) -1)
     (setf (matrix-row m 0) (coerce vec float-vector))
     ;; X3 - X0 = 0
     (setq vec (make-list _dim-invariant-config :initial-element 0))
     (setf (elt vec (eval (joint-index-func "rigid_leg3_x_joint" _invariant-joint-list))) 1)
     (setf (elt vec (eval (joint-index-func "rigid_leg0_x_joint" _invariant-joint-list))) -1)
     (setf (matrix-row m 1) (coerce vec float-vector))
     ;; Y0 - Y1 = 0
     (setq vec (make-list _dim-invariant-config :initial-element 0))
     (setf (elt vec (eval (joint-index-func "rigid_leg0_y_joint" _invariant-joint-list))) 1)
     (setf (elt vec (eval (joint-index-func "rigid_leg1_y_joint" _invariant-joint-list))) -1)
     (setf (matrix-row m 2) (coerce vec float-vector))
     ;; Y2 - Y3 = 0
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
)

;; このパラメータを大きくすると、ベースが小さくなる。今以上の値にすると収束しなくなる。
;; パラメータが小さいと細長いベース形状が出やすいが、
;; 収束可能なギリギリまで大きくすると正方形に近い形状（目的関数を満たす形状）が出やすくなる
;; (defconstant *default-base-size-task-scale* (* 0.1 1e-3))
(defconstant *default-base-size-task-scale* 0)
(unless (boundp '*base-size-task-scale*)
  (setq *base-size-task-scale* *default-base-size-task-scale*))

;; ベースのサイズを小さくするための目的関数の設定
(defmethod instant-configuration-task
  (:_initialize-dimension
   ()
   (setq _dim-theta _num-variant-joint)
   (setq _dim-wrench (* 6 _num-contact))
   (setq _dim-torque _num-drive-joint)
   (setq _dim-phi _num-invariant-joint)
   (setq _dim-variant-config (+ _dim-theta _dim-wrench _dim-torque))
   (setq _dim-invariant-config _dim-phi)
   (setq _dim-config (+ _dim-variant-config _dim-invariant-config))
   (cond (_only-kinematics?
          (setq _dim-task (+ (* 6 _num-kin) _num-posture-joint))
          )
         ;; base size task is added
         ;; 最後に足した数字が、新しく追加する目的関数の要素数
         ((rigid-legs-invariant? _invariant-joint-list)
          (setq _dim-task (+ (* 6 _num-kin) 3 3 _num-drive-joint _num-posture-joint 2)))
         (t
          (setq _dim-task (+ (* 6 _num-kin) 3 3 _num-drive-joint _num-posture-joint))
          ))
   )
  (:base-size-task-value
   (&key (update? t))
   "
ベースのサイズを小さくする+剛体脚の配置を吸着パッドに寄せるための目的関数
"
   ;; ベースサイズを小さくする+剛体脚の配置を吸着パッドに寄せるタスク関数eは6次元ベクトルで、e = *base-size-task-scale* * (X0, X2, Y0, Y2, (X0-X2) , (Y0-Y2))
   ;; なおベースを長方形にするための等式制約により、X0=X3, X2=X1, Y0=Y1, Y2=Y3であることに注意。（8 -> 4変数に削減）
   ;; X0, X2, Y0, Y2は、剛体脚がベース中心から離れすぎないようにするための要素
   ;; (X0-X2), (Y0 - Y2)は、4つの剛体脚が構成する長方形を小さくするための要素
   ;; 目的関数は0.5*e*eであるため、leg0=(X0, Y0)とleg2=(X2, Y2)の位置関係（すなわちX0とX2、Y0とY2の大小関係）は気にしなくて良い
   (let* ((x0 (send (find-joint "rigid_leg0_x_joint" _invariant-joint-list) :joint-angle))
          (y0 (send (find-joint "rigid_leg0_y_joint" _invariant-joint-list) :joint-angle))
          (x2 (send (find-joint "rigid_leg2_x_joint" _invariant-joint-list) :joint-angle))
          (y2 (send (find-joint "rigid_leg2_y_joint" _invariant-joint-list) :joint-angle))
          )
     ;; (scale *base-size-task-scale* (float-vector x0 x2 y0 y2 (- x0 x2) (- y0 y2))))
     ;; (scale *base-size-task-scale* (float-vector (- x0 30) (- x2 -30) (- y0 30) (- y2 -30))))
     (scale *base-size-task-scale* (float-vector (- x0 x2) (- y0 y2))))
   )
  (:base-size-task-jacobian-with-phi
   (&key (update? t))
   ;; ヤコビアンは、タスク関数e = *base-size-task-scale* * (X0, X2, Y0, Y2, (X0-X2) , (Y0-Y2))をX0, Y0, ..., X3, Y3で微分したもの
   ;; なお等式制約により、X0=X3, X2=X1, Y0=Y1, Y2=Y3であることに注意。
   (let* ((jac (make-matrix 2 _num-invariant-joint)) vec
          (x0-index (joint-index-func "rigid_leg0_x_joint" _invariant-joint-list))
          (x1-index (joint-index-func "rigid_leg1_x_joint" _invariant-joint-list))
          (x2-index (joint-index-func "rigid_leg2_x_joint" _invariant-joint-list))
          (x3-index (joint-index-func "rigid_leg3_x_joint" _invariant-joint-list))
          (y0-index (joint-index-func "rigid_leg0_y_joint" _invariant-joint-list))
          (y1-index (joint-index-func "rigid_leg1_y_joint" _invariant-joint-list))
          (y2-index (joint-index-func "rigid_leg2_y_joint" _invariant-joint-list))
          (y3-index (joint-index-func "rigid_leg3_y_joint" _invariant-joint-list))
          )
     ;; ;; X0の微分
     ;; (setq vec (coerce (make-list _num-invariant-joint :initial-element 0) float-vector))
     ;; (setf (elt vec x0-index) 1) ;; X0で微分
     ;; (setf (elt vec x3-index) 1) ;; X3=X0で微分
     ;; (setf (matrix-row jac 0) (scale *base-size-task-scale* vec))
     ;; ;; X2の微分
     ;; (setq vec (coerce (make-list _num-invariant-joint :initial-element 0) float-vector))
     ;; (setf (elt vec x1-index) 1) ;; X1=X2で微分
     ;; (setf (elt vec x2-index) 1) ;; X2で微分
     ;; (setf (matrix-row jac 1) (scale *base-size-task-scale* vec))
     ;; ;; Y0の微分
     ;; (setq vec (coerce (make-list _num-invariant-joint :initial-element 0) float-vector))
     ;; (setf (elt vec y0-index) 1) ;; Y0で微分
     ;; (setf (elt vec y1-index) 1) ;; Y1=Y0で微分
     ;; (setf (matrix-row jac 2) (scale *base-size-task-scale* vec))
     ;; ;; Y2の微分
     ;; (setq vec (coerce (make-list _num-invariant-joint :initial-element 0) float-vector))
     ;; (setf (elt vec y2-index) 1) ;; Y2で微分
     ;; (setf (elt vec y3-index) 1) ;; Y3=Y2で微分
     ;; (setf (matrix-row jac 3) (scale *base-size-task-scale* vec))
     ;; ;; (X0-X2)の微分
     ;; (setq vec (coerce (make-list _num-invariant-joint :initial-element 0) float-vector))
     ;; (setf (elt vec x0-index) 1) ;; X0で微分
     ;; (setf (elt vec x1-index) -1) ;; X1=X2で微分
     ;; (setf (elt vec x2-index) -1) ;; X2で微分
     ;; (setf (elt vec x3-index) 1) ;; X3=X0で微分
     ;; (setf (matrix-row jac 4) (scale *base-size-task-scale* vec))
     ;; ;; (Y0-Y2)の微分
     ;; (setq vec (coerce (make-list _num-invariant-joint :initial-element 0) float-vector))
     ;; (setf (elt vec y0-index) 1) ;; Y0で微分
     ;; (setf (elt vec y1-index) 1) ;; Y1=Y0で微分
     ;; (setf (elt vec y2-index) -1) ;; Y2で微分
     ;; (setf (elt vec y3-index) -1) ;; Y3=Y2で微分
     ;; (setf (matrix-row jac 5) (scale *base-size-task-scale* vec))

     ;; (X0-X2)の微分
     (setq vec (coerce (make-list _num-invariant-joint :initial-element 0) float-vector))
     (setf (elt vec x0-index) 1) ;; X0で微分
     (setf (elt vec x1-index) -1) ;; X1=X2で微分
     (setf (elt vec x2-index) -1) ;; X2で微分
     (setf (elt vec x3-index) 1) ;; X3=X0で微分
     (setf (matrix-row jac 0) (scale *base-size-task-scale* vec))
     ;; (Y0-Y2)の微分
     (setq vec (coerce (make-list _num-invariant-joint :initial-element 0) float-vector))
     (setf (elt vec y0-index) 1) ;; Y0で微分
     (setf (elt vec y1-index) 1) ;; Y1=Y0で微分
     (setf (elt vec y2-index) -1) ;; Y2で微分
     (setf (elt vec y3-index) -1) ;; Y3=Y2で微分
     (setf (matrix-row jac 1) (scale *base-size-task-scale* vec))
     jac)
   )
  (:task-value
   (&key (update? t))
   "
return $\bm{e}(\bm{q})
:= \begin{pmatrix} \bm{e}^{\mathit{kin}}(\bm{q}) \\ \bm{e}^{\mathit{eom\mathchar`-trans}}(\bm{q}) \\ \bm{e}^{\mathit{eom\mathchar`-rot}}(\bm{q}) \\ \bm{e}^{\mathit{trq}}(\bm{q}) \\ \bm{e}^{\mathit{posture}}(\bm{q}) \end{pmatrix}
= \begin{pmatrix} \bm{e}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \\ \bm{e}^{\mathit{eom\mathchar`-trans}}(\bm{\hat{w}}) \\ \bm{e}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}, \bm{\hat{w}}, \bm{\phi}) \\ \bm{e}^{\mathit{trq}}(\bm{\theta}, \bm{\hat{w}}, \bm{\tau}, \bm{\phi}) \\ \bm{e}^{\mathit{posture}}(\bm{\theta}) \end{pmatrix}$
   "
   (concatenate
    float-vector
    (send self :kinematics-task-value :update? update?)
    (if (not _only-kinematics?) (send self :eom-trans-task-value :update? update?))
    (if (not _only-kinematics?) (send self :eom-rot-task-value :update? update?))
    (if (and (not _only-kinematics?) (> _num-drive-joint 0))
        (send self :torque-task-value :update? update?))
    (if (> _num-posture-joint 0)
        (send self :posture-task-value :update? update?))
    ;; base size task added
    (when (rigid-legs-invariant? _invariant-joint-list)
      (send self :base-size-task-value :update? update?))
    )
   )
  (:variant-task-jacobian
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}}{\partial \bm{q}_{\mathit{var}}}
  &=&
  \bordermatrix{
    & N_{\mathit{var\mathchar`-joint}} & 6 N_{\mathit{cnt}} & N_{\mathit{drive\mathchar`-joint}} \cr
    6 N_{\mathit{kin}} & \frac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{\theta}} & & \cr
    3 & & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans}}}{\partial \bm{\hat{w}}} & & \cr
    3 & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\theta}} & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\hat{w}}} & & \cr
    N_{\mathit{drive\mathchar`-joint}} & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\theta}} & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\hat{w}}} & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\tau}} \cr
    N_{\mathit{posture\mathchar`-joint}} & \frac{\partial \bm{e}^{\mathit{posture}}}{\partial \bm{\theta}} & &
  }
\end{eqnarray}

return $\frac{\partial \bm{e}}{\partial \bm{q}_{\mathit{var}}} \in \mathbb{R}^{(6 N_{\mathit{kin}} + 3 + 3 + N_{\mathit{drive\mathchar`-joint}} + N_{\mathit{posture\mathchar`-joint}}) \times (N_{\mathit{var\mathchar`-joint}} + 6 N_{\mathit{cnt}} + N_{\mathit{drive\mathchar`-joint}})}$
   "
   (cond (_variant-task-jacobi
          (fill (array-entity _variant-task-jacobi) 0)
          )
         (t
          (setq _variant-task-jacobi (make-matrix _dim-task _dim-variant-config))
          ))
   (when (= _dim-task 0)
     (return-from :variant-task-jacobian _variant-task-jacobi))
   (copy-block-matrix _variant-task-jacobi
                      (send self :kinematics-task-jacobian-with-theta)
                      0 0)
   (unless _only-kinematics?
     (copy-block-matrix _variant-task-jacobi
                        (send self :eom-trans-task-jacobian-with-wrench)
                        (* 6 _num-kin) _num-variant-joint)
     (copy-block-matrix _variant-task-jacobi
                        (send self :eom-rot-task-jacobian-with-theta)
                        (+ (* 6 _num-kin) 3) 0)
     (copy-block-matrix _variant-task-jacobi
                        (send self :eom-rot-task-jacobian-with-wrench)
                        (+ (* 6 _num-kin) 3) _num-variant-joint)
     (when (> _num-drive-joint 0)
       (copy-block-matrix _variant-task-jacobi
                          (send self :torque-task-jacobian-with-theta)
                          (+ (* 6 _num-kin) 3 3) 0)
       (copy-block-matrix _variant-task-jacobi
                          (send self :torque-task-jacobian-with-wrench)
                          (+ (* 6 _num-kin) 3 3) _num-variant-joint)
       (copy-block-matrix _variant-task-jacobi
                          (send self :torque-task-jacobian-with-torque)
                          (+ (* 6 _num-kin) 3 3) (+ _num-variant-joint (* 6 _num-contact)))
       )
     )
   (when (> _num-posture-joint 0)
     (copy-block-matrix _variant-task-jacobi
                        (send self :posture-task-jacobian-with-theta)
                        (if _only-kinematics?
                            (* 6 _num-kin)
                          (+ (* 6 _num-kin) 3 3 _num-drive-joint))
                        0)
     )
   ;; base-size-task added
   (when (rigid-legs-invariant? _invariant-joint-list)
     (copy-block-matrix _variant-task-jacobi
                        (make-matrix 1 _dim-variant-config)
                        (if _only-kinematics?
                            (* 6 _num-kin)
                          (+ (* 6 _num-kin) 3 3 _num-drive-joint _num-posture-joint)) ;; 36+3+3+0+0=42。最後の要素を置き換え
                        0)
     )
   _variant-task-jacobi
   )
  (:invariant-task-jacobian
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}}{\partial \bm{q}_{\mathit{invar}}}
  &=&
  \bordermatrix{
    & N_{\mathit{invar\mathchar`-joint}} \cr
    6 N_{\mathit{kin}} & \frac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{\phi}} \cr
    3 & \cr
    3 & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\phi}} \cr
    N_{\mathit{drive\mathchar`-joint}} & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\phi}} \cr
    N_{\mathit{posture\mathchar`-joint}} &
  }
\end{eqnarray}

return $\frac{\partial \bm{e}}{\partial \bm{q}_{\mathit{invar}}} \in \mathbb{R}^{(6 N_{\mathit{kin}} + 3 + 3 + N_{\mathit{drive\mathchar`-joint}} + N_{\mathit{posture\mathchar`-joint}}) \times N_{\mathit{invar\mathchar`-joint}}}$
   "
   (cond (_invariant-task-jacobi
          (fill (array-entity _invariant-task-jacobi) 0)
          )
         (t
          (setq _invariant-task-jacobi (make-matrix _dim-task _dim-invariant-config))
          ))
   (when (= _dim-task 0)
     (return-from :invariant-task-jacobian _invariant-task-jacobi))
   (copy-block-matrix _invariant-task-jacobi
                      (send self :kinematics-task-jacobian-with-phi)
                      0 0)
   (unless _only-kinematics?
     (copy-block-matrix _invariant-task-jacobi
                        (send self :eom-rot-task-jacobian-with-phi)
                        (+ (* 6 _num-kin) 3) 0)
     (when (> _num-drive-joint 0)
       (copy-block-matrix _invariant-task-jacobi
                          (send self :torque-task-jacobian-with-phi)
                          (+ (* 6 _num-kin) 3 3) 0)
       )
     )
   ;; base-size-task added
   (when (rigid-legs-invariant? _invariant-joint-list)
     (copy-block-matrix _invariant-task-jacobi
                        (send self :base-size-task-jacobian-with-phi)
                        (if _only-kinematics?
                            (* 6 _num-kin)
                          (+ (* 6 _num-kin) 3 3 _num-drive-joint _num-posture-joint))
                        0)
     )
   _invariant-task-jacobi
   )
  (:print-status
   ()
   "
Print status.
   "
   (when (= _dim-task 0)
     (return-from :print-status nil))
   (warning-message 9 "|task|=~a~%" (norm (send self :task-value :update? nil)))
   (warning-message 9 "|kin-task|=~a:~%" (norm (send self :kinematics-task-value :update? nil)))
   (mapcar
    #'(lambda (kin-task-i)
        (warning-message 9 "  |kin-task-i|=~a~%" (norm kin-task-i)))
    (divide-vector (send self :kinematics-task-value :update? nil) :segment-length 6)
    )
   (unless _only-kinematics?
     (warning-message 9 "|eom-trans-task|=~a~%" (norm (send self :eom-trans-task-value :update? nil)))
     (warning-message 9 "|eom-rot-task|=~a~%" (norm (send self :eom-rot-task-value :update? nil)))
     (warning-message 9 "|torque-task|=~a~%" (norm (send self :torque-task-value :update? nil)))
     )
   (warning-message 9 "|posture-task|=~a~%" (norm (send self :posture-task-value :update? nil)))
   ;; base-size-task added
   (when (rigid-legs-invariant? _invariant-joint-list)
     (warning-message 9 "|base-size-task|=~a~%" (norm (send self :base-size-task-value :update? nil)))
   )
)
