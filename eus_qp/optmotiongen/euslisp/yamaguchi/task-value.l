(defclass instant-configuration-task
  (:posture-task-value
   (&key (update? t))
   "
ベースのサイズを小さくするための制約
   "
   (map float-vector
        #'(lambda (j a sc)
            (* sc (send j :angle-to-speed (- a (send j :joint-angle)))))
        _posture-joint-list
        _posture-joint-angle-list
        _target-posture-scale-list
        )
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
    (send self :base-size-task-value :update? update?)
    )
   )
)
