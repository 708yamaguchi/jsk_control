(load "../inverse-kinematics-wrapper.l")
(load "../discrete-kinematics-configuration-task.l")
(load "../sqp-msc-optimization.l")
(require "models/arrow-object.l")
(require "models/single-arrow-object.l")

(load "room610-model-list.l") ;; Set *room610-model-list*
(load "attachable-plane.l")
(load "yamaguchi-arm.l") ;; Define (yamaguchi-arm) function

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
      (send robot :newcoords (make-coords))
      (let ((normal (send task-plane :normal))
            (offset (send task-plane :offset)))
        (when (eq normal :x+)
          (send robot :rotate pi/2 :y)
          (send robot :locate (float-vector offset 0 0) :world))
        (when (eq normal :x-)
          (send robot :rotate -pi/2 :y)
          (send robot :locate (float-vector offset 0 0) :world))
        (when (eq normal :y+)
          (send robot :rotate -pi/2 :x)
          (send robot :locate (float-vector 0 offset 0) :world))
        (when (eq normal :y-)
          (send robot :rotate pi/2 :x)
          (send robot :locate (float-vector 0 offset 0) :world))
        (when (eq normal :z+)
          (send robot :locate (float-vector 0 0 offset) :world))
        (when (eq normal :z-)
          (send robot :rotate pi :x)
          (send robot :locate (float-vector 0 0 offset) :world)))

      ;; 3. optimization
      ;; TODO: dotimesで他のタスクがあるかどうかで、:inverse-kinematics-optmotiongenの成否が変わる
      (send robot
            :inverse-kinematics-optmotiongen
            (send task-arrow :copy-worldcoords)
            :move-target (send robot :end-coords)
            :translation-axis t
            :rotation-axis nil
            :collision-avoidance-link-pair (list
                                            (list (elt (send robot :links) 0) (elt (send robot :links) 3))
                                            (list (elt (send robot :links) 0) (car (last (send robot :links))))
                                            )
            :collision-distance-limit -10 ;; 多少のめり込みを許容する？と、IKの成功率が格段に上がる[mm]
            :debug-view t
            :root-virtual-mode
            (let ((normal (send task-plane :normal)) mode)
              (when (or (eq normal :x+) (eq normal :x-)) (setq mode :planar-x))
              (when (or (eq normal :y+) (eq normal :y-)) (setq mode :planar-y))
              (when (or (eq normal :z+) (eq normal :z-)) (setq mode :planar-z))
              mode)
            ;; The last added element is rotation joint on the plane (continuous rotation)
            :root-virtual-joint-min-angle-list
            (append (send task-plane :min-angle-list) (list -1e10))
            :root-virtual-joint-max-angle-list
            (append (send task-plane :max-angle-list) (list 1e10))
            :pre-process-func pre-process-func
            :post-process-func post-process-func)
      ))
  nil)
(warn "(main-with-loop)~%")

(defun main-with-trajectory
    (&key
     (pre-process-func)
     (post-process-func)
     (visualize-callback-func)
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
  (setq *robot* (yamaguchi-arm))

  (let* ((task-size (length *task-arrow-list*)) robot robot-list
         (ret
          ;; 2. optimization
          ;; TODO: dotimesで他のタスクがあるかどうかで、:inverse-kinematics-optmotiongenの成否が変わる
          (send *robot*
                :inverse-kinematics-trajectory-optmotiongen
                (send-all *task-arrow-list* :copy-worldcoords)
                :move-target-list (send *robot* :end-coords)
                :translation-axis-list (make-list task-size :initial-element t)
                :rotation-axis-list (make-list task-size :initial-element nil)
                :collision-avoidance-link-pair (list
                                                (list (elt (send *robot* :links) 0) (elt (send *robot* :links) 3))
                                                (list (elt (send *robot* :links) 0) (car (last (send *robot* :links))))
                                                )
                :collision-distance-limit -10 ;; 多少のめり込みを許容する？と、IKの成功率が格段に上がる[mm]
                :attachable-plane-list *attachable-plane-list*
                :debug-view t
                :adjacent-regular-scale 0.0
                :pre-process-func pre-process-func
                :post-process-func post-process-func)))

    ;; 3. Visualize result
    (when ret
      (dolist (r ret)
        (setq robot (copy-object *robot*))
        (send robot :angle-vector (cadr (car r)))
        (send robot :newcoords (cadr (cadr r)))
        (setq robot-list (append robot-list (list robot))))
      (objects (append *room610-model-list*
                       (send-all *attachable-plane-list* :object)
                       ;; (send-all *attachable-plane-list* :normal-object) ;; For debug
                       *task-arrow-list*
                       robot-list))
      (visualize-trajectory-result ret :visualize-callback-func visualize-callback-func))
    ret))
(warn "(main-with-trajectory)~%")
