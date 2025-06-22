(load "../inverse-kinematics-statics-wrapper.l")
(load "../discrete-kinematics-configuration-task.l")
(load "../sqp-msc-optimization.l")
(require "models/arrow-object.l")
(require "models/single-arrow-object.l")

(load "room610-model-list.l") ;; Set *room610-model-list*
(load "attachable-plane.l")
(load "yamaguchi-arm.l") ;; Define (yamaguchi-arm) function
(load "contact-constrant-models.l")
(load "base-shape-constraint.l") ;; Add equality constraint for base shape

;; TODO: Create edge case test function

;; 1. Prepare task environment
;; If you change task configuration, change only this section.
;; The following variables are the same length and correspond in order
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
(setq *task-coords-list*
      (list
       ;; 蛇口
       (send (instance arrow-object :init) :newcoords
             (make-coords :pos (float-vector 8200 800 1000) :rpy (list (deg2rad 60) 0 0)))
       ;; IH
       (send (instance arrow-object :init) :newcoords
             (make-coords :pos (float-vector 8200 -800 1000) :rpy (list 0 0 0)))
       ;; 中央の机
       (send (instance arrow-object :init) :newcoords
             (make-coords :pos (float-vector 4600 300 720) :rpy (list 0 0 0)))
       ;; 電子レンジ
       (send (instance arrow-object :init) :newcoords
             (make-coords :pos (float-vector 6200 1800 1200) :rpy (list 0 0 0)))
       ;; 冷蔵庫
       (send (instance arrow-object :init) :newcoords
             (make-coords :pos (float-vector 7000 2400 1850) :rpy (list 0 0 0)))
       ))
;; TODO: *task-coords-list*の代わりにこちらをvisualizeする
(setq *task-wrench-list*
      (list
       (float-vector 0.1 0 -1 0 0 0)
       (float-vector 0.1 0 -1 0 0 0)
       (float-vector 0.1 0 -1 0 0 0)
       (float-vector 0.1 0 -1 0 0 0)
       (float-vector 0.1 0 -1 0 0 0)
       ))
(unless (and (eq (length *attachable-plane-list*) (length *task-coords-list*))
             (eq (length *task-coords-list*) (length *task-wrench-list*)))
  (print "Task definition is wrong.")
  (exit))

;; 前までは、virtual linkへの制約によって脱着ベースと壁面を平行にしていた。
;; この例では、脱着ベースの剛体脚と吸着パッドが壁面上にあるという制約をかけることで、脱着ベースと壁面を平行にしている
(defun main-without-statics-loop
    (&key
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot-list* nil)
  (let ((robot (yamaguchi-arm)))
    (dotimes (i (length *attachable-plane-list*))
      (setq *robot-list* (append *robot-list*
                                 (list (copy-object robot))))))
  (objects (append *room610-model-list*
                   (send-all *attachable-plane-list* :object)
                   ;; (send-all *attachable-plane-list* :normal-object) ;; For debug
                   *task-coords-list*
                   *robot-list*))
  (set-room610-viewpoint)

  (dotimes (i (length *robot-list*))
    (let* ((robot (elt *robot-list* i))
           (task-plane (elt *attachable-plane-list* i))
           (task-plane-face (send task-plane :face :margin 0))
           (task-arrow (elt *task-coords-list* i)))
      ;; 2. Rotate robot along task-plane
      ;; It is assumed that robot and room610 coords are the same at first.
      (send robot :newcoords (make-coords))
      ;; 3. optimization
      (send robot
            ;; メモ：第一引数target-coords、:move-target、:translation-axis、:rotation-axisは個数と順番を一致させる
            :inverse-kinematics-optmotiongen
            (list (send task-arrow :copy-worldcoords)
                  task-plane-face
                  task-plane-face
                  task-plane-face
                  task-plane-face
                  task-plane-face
                  )
            :move-target
            (list (send robot :end-coords)
                  (send robot :get :rigid-leg0-coords)
                  (send robot :get :rigid-leg1-coords)
                  (send robot :get :rigid-leg2-coords)
                  (send robot :get :rigid-leg3-coords)
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

(defun main-without-statics-trajectory
    (&key
     (pre-process-func)
     (post-process-func)
     (visualize-callback-func)
     &allow-other-keys
     )
  (let* ((task-size (length *task-coords-list*))
         (additional-draw-objects
          (append *room610-model-list*
                  (send-all *attachable-plane-list* :object)
                  ;; (send-all *attachable-plane-list* :normal-object) ;; For debug
                  *task-coords-list*)))
    ;; 2. Visualize
    (unless (boundp '*irtviewer*)
      (make-irtviewer))
    (send *irtviewer* :objects additional-draw-objects)
    (set-room610-viewpoint)
    (send *irtviewer* :draw-objects)
    ;; 3. optimization
    (setq *robot* (yamaguchi-arm))
    ;; 2個以上のrigid-legに対してtarget-coordsをfaceで指定することで、平面に拘束を書けられる
    (let* ((move-target (list (send *robot* :end-coords)
                              (send *robot* :get :rigid-leg0-coords)
                              (send *robot* :get :rigid-leg1-coords)
                              (send *robot* :get :rigid-leg2-coords)
                              (send *robot* :get :rigid-leg3-coords)
                              (send *robot* :get :vacuum-pad-coords)
                              ))
           (limb-size (1- (length move-target)))
           (task-target-coords (send-all *task-coords-list* :copy-worldcoords))
           (translation-axis-list (make-list task-size :initial-element
                                             (append (list t) (make-list limb-size :initial-element nil))))
           ;; (translation-axis-list (make-list task-size :initial-element
           ;;                                   (append (list nil) (make-list limb-size :initial-element nil))))
           (rotation-axis-list (make-list task-size :initial-element
                                 (append (list nil) (make-list limb-size :initial-element nil))))
           ret target-coords-list move-target-list)
      (dotimes (i task-size)
        (setq target-coords-list
              (append target-coords-list
                      (list (append (list (elt task-target-coords i))
                                    (make-list limb-size :initial-element
                                               (send (elt *attachable-plane-list* i) :face :margin 0))))))
        (setq move-target-list (append move-target-list (list move-target)))
        )
      (setq ret (send *robot*
                      :inverse-kinematics-trajectory-optmotiongen
                      target-coords-list
                      :move-target-list move-target-list
                      :translation-axis-list translation-axis-list
                      :rotation-axis-list rotation-axis-list
                      ;; :collision-avoidance-link-pair (let ((base-cube (send *robot* :base-cube)))
                      ;;                                  (send *robot* :assoc base-cube)
                      ;;                                  (list
                      ;;                                   (list base-cube (elt (send *robot* :links) 2))
                      ;;                                   (list base-cube (elt (send *robot* :links) 3))
                      ;;                                   (list base-cube (car (last (send *robot* :links))))
                      ;;                                   ))
                      ;; :collision-distance-limit 5
                      ;; :attachable-plane-list *attachable-plane-list*
                      :root-virtual-mode :6dof
                      :optimize-torque? nil
                      :debug-view (list :additional-draw-objects additional-draw-objects)
                      :adjacent-regular-scale 0.0
                      :pre-process-func pre-process-func
                      :post-process-func post-process-func))
      ;; (visualize-trajectory-result ret :visualize-callback-func visualize-callback-func)
      ret)
    ))
(warn "(main-without-statics-trajectory)~%")

(defun main-with-loop
    (&key
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot-list* nil)
  (let ((robot (yamaguchi-arm)))
    (dotimes (i (length *attachable-plane-list*))
      (setq *robot-list* (append *robot-list*
                                 (list (copy-object robot))))))
  (objects (append *room610-model-list*
                   (send-all *attachable-plane-list* :object)
                   ;; (send-all *attachable-plane-list* :normal-object) ;; For debug
                   *task-coords-list*
                   *robot-list*))
  (set-room610-viewpoint)

  (dotimes (i (length *robot-list*))
    (let* ((robot (elt *robot-list* i))
           (task-plane (elt *attachable-plane-list* i))
           (task-plane-face (send task-plane :face :margin 0)) ;; Note: we need to the same face object for the arguments
           (task-arrow (elt *task-coords-list* i)))
      ;; 2. Rotate robot along task-plane
      ;; It is assumed that robot and room610 coords are the same at first.
      (setq *robot* robot) ;; (draw-force-value) function uses *robot* variable...
      (send robot :newcoords (make-coords))
      ;; 3. optimization
      (send robot
            ;; メモ：第一引数target-coords、:move-target、:translation-axis、:rotation-axisは個数と順番を一致させる
            :inverse-kinematics-statics-optmotiongen
            (list (send task-arrow :copy-worldcoords)
                  task-plane-face
                  task-plane-face
                  task-plane-face
                  task-plane-face
                  task-plane-face
                  )
            :move-target
            (list (send robot :end-coords)
                  (send robot :get :rigid-leg0-coords)
                  (send robot :get :rigid-leg1-coords)
                  (send robot :get :rigid-leg2-coords)
                  (send robot :get :rigid-leg3-coords)
                  (send robot :get :vacuum-pad-coords)
                  )
            ;; メモ：引数:contact-target-coords、:contact-move-target、:contact-constraintも個数と順番を一致させる
            :contact-target-coords
            (list task-plane-face
                  task-plane-face
                  task-plane-face
                  task-plane-face
                  task-plane-face
                  )
            :contact-move-target
            (list (send robot :get :rigid-leg0-coords)
                  (send robot :get :rigid-leg1-coords)
                  (send robot :get :rigid-leg2-coords)
                  (send robot :get :rigid-leg3-coords)
                  (send robot :get :vacuum-pad-coords)
                  )
            ;; NOTE: 制約を入れて動くことを確認。ただし制約がすごく緩いので、あまり意味はない
            :contact-constraint
            (list (instance norm-contact-constraint :init :fz)
                  (instance norm-contact-constraint :init :fz)
                  (instance norm-contact-constraint :init :fz)
                  (instance norm-contact-constraint :init :fz)
                  (instance norm-contact-constraint :init :fz)
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

;; これが研究で使う関数。吸着ベースの設計パラメータを時不変変数として扱っている。
(defun main-with-trajectory
    (&key
     (pre-process-func)
     (post-process-func)
     (visualize-callback-func)
     &allow-other-keys
     )
  (let* ((additional-draw-objects
          (append *room610-model-list*
                  (send-all *attachable-plane-list* :object)
                  ;; (send-all *attachable-plane-list* :normal-object) ;; For debug
                  *task-coords-list*)))
    ;; 2. Visualize
    (unless (boundp '*irtviewer*)
      (make-irtviewer))
    (send *irtviewer* :objects additional-draw-objects)
    (set-room610-viewpoint)
    (send *irtviewer* :draw-objects)
    ;; 3. optimization
    (setq *robot* (yamaguchi-arm))
    (send *robot* :leg-init :pos 50)
    (let* ((task-size (length *task-coords-list*))
           (move-target (list (send *robot* :end-coords)
                              (send *robot* :get :rigid-leg0-coords)
                              (send *robot* :get :rigid-leg1-coords)
                              (send *robot* :get :rigid-leg2-coords)
                              (send *robot* :get :rigid-leg3-coords)
                              (send *robot* :get :vacuum-pad-coords)
                              ))
           (limb-size (1- (length move-target)))
           (task-target-coords (send-all *task-coords-list* :copy-worldcoords))
           contact-target-coords-list
           contact-move-target-list
           (translation-axis-list (make-list task-size :initial-element
                                             (append (list t) (make-list limb-size :initial-element :z))))
           (rotation-axis-list (make-list task-size :initial-element
                                          (append (list nil) (make-list limb-size :initial-element :z))))
           (collision-avoidance-link-pair (list
                                             (list (elt (send *robot* :links) 0) (elt (send *robot* :links) 2))
                                             (list (elt (send *robot* :links) 0) (elt (send *robot* :links) 3))
                                             (list (elt (send *robot* :links) 0) (car (last (send *robot* :links))))
                                             ))
           sqp-opt target-coords-list move-target-list contact-constraint-list
           (external-coords-list (make-list task-size :initial-element (list (send *robot* :end-coords))))
           (external-wrench-list (mapcar #'list *task-wrench-list*))
           )
      (dotimes (i task-size)
        ;; メモ：各平面に固有の名前を付けないと、inverse-kinematics-statics-wrapper.lの以下のコードで除去される
        ;; (remove-duplicates (flatten target-coords-list) :test #'equal)
        (let (task-face)
          (dotimes (j limb-size)
            (setq task-face (append task-face (list (send (elt *attachable-plane-list* i)
                                                          :face :margin 10 :name (format nil "~A,~A" i j))))))
          (setq target-coords-list
                (append target-coords-list
                        (list (append (list (elt task-target-coords i))
                                      task-face)))))
        (setq move-target-list (append move-target-list (list move-target)))
        (setq contact-constraint-list (append contact-constraint-list
                                              (list
                                               (list
                                                (instance default-contact-constraint
                                                          :init-from-support-polygon
                                                          :support-polygon nil
                                                          :mu-trans 0.2
                                                          :mu-rot 0.002)
                                                (instance default-contact-constraint
                                                          :init-from-support-polygon
                                                          :support-polygon nil
                                                          :mu-trans 0.2
                                                          :mu-rot 0.002)
                                                (instance default-contact-constraint
                                                          :init-from-support-polygon
                                                          :support-polygon nil
                                                          :mu-trans 0.2
                                                          :mu-rot 0.002)
                                                (instance default-contact-constraint
                                                          :init-from-support-polygon
                                                          :support-polygon nil
                                                          :mu-trans 0.2
                                                          :mu-rot 0.002)
                                                (instance vacuum-pad-contact-constraint :init)
                                                )
                                               )))
        )
      (setq contact-target-coords-list (mapcar #'cdr target-coords-list))
      (setq contact-move-target-list (mapcar #'cdr move-target-list))
      (setq sqp-opt (send *robot*
                          :inverse-kinematics-statics-trajectory-optmotiongen
                          target-coords-list
                          :move-target-list move-target-list
                          :contact-target-coords-list contact-target-coords-list
                          :contact-move-target-list contact-move-target-list
                          :contact-constraint-list contact-constraint-list
                          ;; メモ：external-wrench-listは、手先が発揮しているレンチではなく、手先が受けているレンチ
                          :external-wrench-list external-wrench-list
                          :external-coords-list external-coords-list
                          :translation-axis-list translation-axis-list
                          :rotation-axis-list rotation-axis-list
                          ;; :collision-avoidance-link-pair collision-avoidance-link-pair
                          ;; :collision-distance-limit -0.5
                          :invariant-joint-list (send *robot* :invariant-joint-list)
                          :root-virtual-mode :6dof
                          :optimize-torque? nil
                          :stop 50
                          :min-loop 30
                          ;; :stop 100
                          ;; :min-loop 100
                          :debug-view (list :additional-draw-objects additional-draw-objects)
                          :adjacent-regular-scale 0.0
                          :pre-process-func pre-process-func
                          :post-process-func post-process-func))
      ;; (visualize-trajectory-result ret :visualize-callback-func visualize-callback-func)

      ;; 4. Print result
      (let* ((traj-config-task (send sqp-opt :config-task))
             (instant-config-task-list (send traj-config-task :instant-config-task-list)))
        (format t "~%===== Result (in world coords) =====~%")
        (dotimes (i (length instant-config-task-list))
          (format t "task~A~%" i)
          (let* ((config-task (elt instant-config-task-list i))
                 (contact-attention-coords-list (send config-task :contact-attention-coords-list))
                 (contact-target-coords-list (send config-task :contact-target-coords-list)))
            (format t "angle vector~%")
            (format t " ~A~%" (send (send config-task :robot-env) :robot :angle-vector))
            (format t "base size~%")
            (let* ((robot (send (send config-task :robot-env) :robot))
                   (x0 (send (send robot :leg-joint 0 :x) :joint-angle))
                   (y0 (send (send robot :leg-joint 0 :y) :joint-angle))
                   (x2 (send (send robot :leg-joint 2 :x) :joint-angle))
                   (y2 (send (send robot :leg-joint 2 :y) :joint-angle)))
              (format t " X size: ~A~%" (abs (- x0 x2)))
              (format t " Y size: ~A~%" (abs (- y0 y2))))
            (format t "worldcoords~%")
            (format t " ~A~%" (send (send config-task :robot-env) :robot :copy-worldcoords))
            ;; Print contact coords
            (format t "Contact coords~%")
            (dolist (contact-target-coords contact-target-coords-list)
              (format t " ~A~%" contact-target-coords))
            ;; Print wrench
            (dolist (force-type (list :force-list :moment-list :external-force-list :external-moment-list))
              (format t " ~A " force-type)
              (if (or (eq force-type :force-list) (eq force-type :external-force-list))
                  (format t "[N]~%"))
              (if (or (eq force-type :moment-list) (eq force-type :external-moment-list))
                  (format t "[Nm]~%"))
              (if (send config-task force-type)
                  (progn
                    (when (or (eq force-type :force-list) (eq force-type :moment-list))
                      (dotimes (j (length contact-attention-coords-list))
                        (format t "   ~A: ~A~%" (elt (send-all contact-attention-coords-list :name) j)
                                (elt (send config-task force-type) j))))
                    (when (or (eq force-type :external-force-list) (eq force-type :external-moment-list))
                      (dotimes (j (length (send config-task :external-coords-list)))
                        (format t "   ~A: ~A~%" (send (elt (send config-task :external-coords-list) j) :name)
                                (elt (send config-task force-type) j))
                        )))
                (format t "   nil~%"))))
          (format t "~%")))
      (format t "Status: ~A~%==================~%~%" (send sqp-opt :status))
      sqp-opt)))
(warn "(main-with-trajectory)~%")
