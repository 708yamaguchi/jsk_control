(load "../inverse-kinematics-wrapper.l")
(load "../discrete-kinematics-configuration-task.l")
(load "../sqp-msc-optimization.l")
;; (require "irteus/demo/sample-arm-model.l")
;; (require "irteus/demo/sample-robot-model.l")
(require "models/arrow-object.l")

(load "yamaguchi_6axis_arm_with_vacuum_base.l")

(defmethod yamaguchi_6axis_arm_with_vacuum_base-robot
  (:end-coords ()
    (send self :rarm :end-coords)))

(defun sample-yamaguchi-arm-reach-ik-raw-msc
    (&key
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 400 -200 300) :rpy (list (deg2rad 60) 0 0)))
  ;; (setq *robot* (instance sarmclass :init))
  (setq *robot* (instance yamaguchi_6axis_arm_with_vacuum_base-robot :init))
  (setq *contact-list*
        (list
         (instance contact-ik-arg :init
                   :name "target-coords"
                   :target-coords (send *arrow* :copy-worldcoords)
                   :translation-axis t
                   :rotation-axis nil
                   )
         ))
  (setq *robot-env*
        (instance robot-environment :init
                  :robot *robot*
                  :contact-list *contact-list*
                  :root-virtual-mode :fix
                  ))

  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list nil
                  :only-kinematics? t
                  :kin-target-coords-list
                  (list (send (send *robot-env* :contact "target-coords") :contact-coords))
                  :kin-attention-coords-list
                  (list (send *robot-env* :robot :end-coords))
                  ))

  (setq *sqp-opt*
        (instance sqp-msc-optimization :init
                  :config-task *instant-config-task*
                  :pre-process-func pre-process-func
                  :post-process-func post-process-func
                  ))
  (objects (append (list *arrow*) (send-all (send *sqp-opt* :config-task-list) :robot-env)))
  (send *sqp-opt* :optimize :loop-num 50)
  t)
(warn "(sample-yamaguchi-arm-reach-ik-raw-msc)~%")

(defun sample-yamaguchi-arm-reach-ik-with-root-virtual-joint
    (&key
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  ;; (setq *robot* (instance sarmclass :init))
  (setq *robot* (instance yamaguchi_6axis_arm_with_vacuum_base-robot :init))
  (send *robot* :rotate pi/2 :x)
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 1400 -200 300) :rpy (list (deg2rad 60) 0 0)))
  (objects (list *robot* *arrow*))

  (send *robot*
        :inverse-kinematics-optmotiongen
        (send *arrow* :copy-worldcoords)
        :move-target (send *robot* :end-coords)
        :translation-axis t
        :rotation-axis t
        :debug-view t
        :root-virtual-mode :planar-y
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
(warn "(sample-yamaguchi-arm-reach-ik-with-root-virtual-joint)~%")
