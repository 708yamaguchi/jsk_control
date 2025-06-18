(load "../inverse-kinematics-statics-wrapper.l")
(load "../discrete-kinematics-configuration-task.l")
(load "../sqp-msc-optimization.l")
(require "models/arrow-object.l")
(require "models/single-arrow-object.l")

(load "room610-model-list.l") ;; Set *room610-model-list*
(load "attachable-plane.l")
(load "yamaguchi-arm.l") ;; Define (yamaguchi-arm) function
(load "contact-constrant-models.l")

;; TODO: Create edge case test function

;; 前までは、virtual linkへの制約によって脱着ベースと壁面を平行にしていた。
;; この例では、脱着ベースの剛体脚と吸着パッドが壁面上にあるという制約をかけることで、脱着ベースと壁面を平行にしている
(defun main-without-statics-loop
    (&key
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  ;; 1. Prepare task environment
  ;; If you change task configuration, change only this section.
  ;; *robot-list*, *attachable-plane-list* and *task-arrow-list* are the same length and correspond in order
  (setq *attachable-plane-list*
        (list
         (instance attachable-plane :init
                   :offset 900
                   :min-angle-list (list 7800 -250)
                   :max-angle-list (list 8300 600)
                   :normal :z+)
         (instance attachable-plane :init
                   :offset 8500
                   :min-angle-list (list -1250 1100)
                   :max-angle-list (list 1600 2000)
                   :normal :x-)
         (instance attachable-plane :init
                   :offset 720
                   :min-angle-list (list 4300 -700)
                   :max-angle-list (list 5000 150)
                   :normal :z+)
         (instance attachable-plane :init
                   :offset 1200
                   :min-angle-list (list 6050 1870)
                   :max-angle-list (list 6450 2150)
                   :normal :z+)
         (instance attachable-plane :init
                   :offset 1850
                   :min-angle-list (list 6720 2010)
                   :max-angle-list (list 7300 2520)
                   :normal :z+)
         ))
  (setq *task-arrow-list*
        (list
         ;; 蛇口
         (send (instance arrow-object :init) :newcoords
               (make-coords :pos (float-vector 8200 800 1000) :rpy (list (deg2rad 60) 0 0)))
         ;; IH
         (send (instance arrow-object :init) :newcoords
               (make-coords :pos (float-vector 8200 -800 1000) :rpy (list 0 0 0)))
         ;; 中央の机
         (send (instance arrow-object :init) :newcoords
               (make-coords :pos (float-vector 4600 350 720) :rpy (list 0 0 0)))
         ;; 電子レンジ
         (send (instance arrow-object :init) :newcoords
               (make-coords :pos (float-vector 6200 1800 1200) :rpy (list 0 0 0)))
         ;; 冷蔵庫
         (send (instance arrow-object :init) :newcoords
               (make-coords :pos (float-vector 7000 2400 1850) :rpy (list 0 0 0)))
         ))
  (setq *robot-list* nil)
  (let ((robot (yamaguchi-arm)))
    (dotimes (i (length *attachable-plane-list*))
      (setq *robot-list* (append *robot-list*
                                 (list (copy-object robot))))))
  (objects (append *room610-model-list*
                   (send-all *attachable-plane-list* :object)
                   ;; (send-all *attachable-plane-list* :normal-object) ;; For debug
                   *task-arrow-list*
                   *robot-list*))

  (dotimes (i (length *robot-list*))
    (let ((robot (elt *robot-list* i))
          (task-plane (elt *attachable-plane-list* i))
          (task-arrow (elt *task-arrow-list* i)))
      ;; 2. Rotate robot along task-plane
      ;; It is assumed that robot and room610 coords are the same at first.
      (send robot :newcoords (make-coords))
      ;; 3. optimization
      (send robot
            ;; メモ：第一引数target-coords、:move-target、:translation-axis、:rotation-axisは個数と順番を一致させる
            :inverse-kinematics-optmotiongen
            (list (send task-arrow :copy-worldcoords)
                  (send task-plane :face 0)
                  (send task-plane :face 0)
                  (send task-plane :face 0)
                  (send task-plane :face 0)
                  (send task-plane :face 0)
                  )
            :move-target
            (list (send robot :end-coords)
                  (send robot :get :rigid-leg1-coords)
                  (send robot :get :rigid-leg2-coords)
                  (send robot :get :rigid-leg3-coords)
                  (send robot :get :rigid-leg4-coords)
                  (send robot :get :vacuum-pad-coords)
                  )
            :translation-axis (list t nil nil nil nil nil)
            :rotation-axis (list nil nil nil nil nil nil)
            :collision-avoidance-link-pair (list
                                            (list (elt (send robot :links) 0) (elt (send robot :links) 3))
                                            (list (elt (send robot :links) 0) (car (last (send robot :links))))
                                            )
            :collision-distance-limit -10 ;; 多少のめり込みを許容する？と、IKの成功率が格段に上がる[mm]
            :debug-view t
            :root-virtual-mode :6dof
            :optimize-torque? nil
            :pre-process-func pre-process-func
            :post-process-func post-process-func)
      ))
  nil)
(warn "(main-without-statics-loop)~%")

(defun main-with-loop
    (&key
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  ;; 1. Prepare task environment
  ;; If you change task configuration, change only this section.
  ;; *robot-list*, *attachable-plane-list* and *task-arrow-list* are the same length and correspond in order
  (setq *attachable-plane-list*
        (list
         (instance attachable-plane :init
                   :offset 900
                   :min-angle-list (list 7800 -250)
                   :max-angle-list (list 8300 600)
                   :normal :z+)
         (instance attachable-plane :init
                   :offset 8500
                   :min-angle-list (list -1250 1100)
                   :max-angle-list (list 1600 2000)
                   :normal :x-)
         (instance attachable-plane :init
                   :offset 720
                   :min-angle-list (list 4300 -700)
                   :max-angle-list (list 5000 150)
                   :normal :z+)
         (instance attachable-plane :init
                   :offset 1200
                   :min-angle-list (list 6050 1870)
                   :max-angle-list (list 6450 2150)
                   :normal :z+)
         (instance attachable-plane :init
                   :offset 1850
                   :min-angle-list (list 6720 2010)
                   :max-angle-list (list 7300 2520)
                   :normal :z+)
         ))
  (setq *task-arrow-list*
        (list
         ;; 蛇口
         (send (instance arrow-object :init) :newcoords
               (make-coords :pos (float-vector 8200 800 1000) :rpy (list (deg2rad 60) 0 0)))
         ;; IH
         (send (instance arrow-object :init) :newcoords
               (make-coords :pos (float-vector 8200 -800 1000) :rpy (list 0 0 0)))
         ;; 中央の机
         (send (instance arrow-object :init) :newcoords
               (make-coords :pos (float-vector 4600 350 720) :rpy (list 0 0 0)))
         ;; 電子レンジ
         (send (instance arrow-object :init) :newcoords
               (make-coords :pos (float-vector 6200 1800 1200) :rpy (list 0 0 0)))
         ;; 冷蔵庫
         (send (instance arrow-object :init) :newcoords
               (make-coords :pos (float-vector 7000 2400 1850) :rpy (list 0 0 0)))
         ))
  (setq *robot-list* nil)
  (let ((robot (yamaguchi-arm)))
    (dotimes (i (length *attachable-plane-list*))
      (setq *robot-list* (append *robot-list*
                                 (list (copy-object robot))))))
  (objects (append *room610-model-list*
                   (send-all *attachable-plane-list* :object)
                   ;; (send-all *attachable-plane-list* :normal-object) ;; For debug
                   *task-arrow-list*
                   *robot-list*))

  (dotimes (i (length *robot-list*))
    (let ((robot (elt *robot-list* i))
          (task-plane (elt *attachable-plane-list* i))
          (task-arrow (elt *task-arrow-list* i)))
      ;; 2. Rotate robot along task-plane
      ;; It is assumed that robot and room610 coords are the same at first.
      (setq *robot* robot) ;; (draw-force-value) function uses *robot* variable...
      (send robot :newcoords (make-coords))
      ;; 3. optimization
      (send robot
            ;; メモ：第一引数target-coords、:move-target、:translation-axis、:rotation-axisは個数と順番を一致させる
            :inverse-kinematics-statics-optmotiongen
            ;; :inverse-kinematics-optmotiongen
            (list (send task-arrow :copy-worldcoords)
                  (send task-plane :face :margin 0)
                  (send task-plane :face :margin 0)
                  (send task-plane :face :margin 0)
                  (send task-plane :face :margin 0)
                  (send task-plane :face :margin 0)
                  )
            :move-target
            (list (send robot :end-coords)
                  (send robot :get :rigid-leg1-coords)
                  (send robot :get :rigid-leg2-coords)
                  (send robot :get :rigid-leg3-coords)
                  (send robot :get :rigid-leg4-coords)
                  (send robot :get :vacuum-pad-coords)
                  )
            ;; メモ：引数:contact-target-coords、:contact-move-target、:contact-constraintも個数と順番を一致させる
            ;; TODO: アームだけでなく、剛体脚と吸着パッドの接触制約も入れないとIKが失敗しそう。
            ;; TODO: 一方で、これらを入れるとSegmentation faultする。baseから剛体脚へのjoint-listが必要？
            :contact-target-coords
            (list (send task-arrow :copy-worldcoords)
                  ;; (send task-plane :face :margin 0)
                  ;; (send task-plane :face :margin 0)
                  ;; (send task-plane :face :margin 0)
                  ;; (send task-plane :face :margin 0)
                  ;; (send task-plane :face :margin 0)
                  )
            :contact-move-target
            (list (send robot :end-coords)
                  ;; (send robot :get :rigid-leg1-coords)
                  ;; (send robot :get :rigid-leg2-coords)
                  ;; (send robot :get :rigid-leg3-coords)
                  ;; (send robot :get :rigid-leg4-coords)
                  ;; (send robot :get :vacuum-pad-coords)
                  )
            :contact-constraint
            (list (instance hand-contact-constraint :init :torque (float-vector 0 0 0))
                  ;; (instance rigid-leg-contact-constraint :init)
                  ;; (instance rigid-leg-contact-constraint :init)
                  ;; (instance rigid-leg-contact-constraint :init)
                  ;; (instance rigid-leg-contact-constraint :init)
                  ;; (instance vacuum-pad-contact-constraint :init)
                  )
            :translation-axis (list t nil nil nil nil nil)
            :rotation-axis (list nil nil nil nil nil nil)
            :collision-avoidance-link-pair (list
                                            (list (elt (send robot :links) 0) (elt (send robot :links) 3))
                                            (list (elt (send robot :links) 0) (car (last (send robot :links))))
                                            )
            :collision-distance-limit -10 ;; 多少のめり込みを許容する？と、IKの成功率が格段に上がる[mm]
            :debug-view t
            :root-virtual-mode :6dof
            :optimize-torque? nil
            :pre-process-func pre-process-func
            :post-process-func post-process-func)
      ))
  nil)
(warn "(main-with-loop)~%")

