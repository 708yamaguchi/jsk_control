(load "../inverse-kinematics-statics-wrapper.l")
(load "../discrete-kinematics-configuration-task.l")
(load "../sqp-msc-optimization.l")

(load "room610-model-list.l") ;; Set *room610-model-list*
(load "task.l") ;; Set *tasks*
(load "yamaguchi-arm.l") ;; Define (yamaguchi-arm) function
(load "contact-constrant-models.l")
(load "base-shape-constraint.l") ;; Add equality constraint for base shape

;; TODO：ベースの形状を固定にして:inverse-kinematics-statics-optmotiongenを解いて、静力学的に解が出るかをチェックする
;; TODO：現実と同じ物理パラメータになっているかをチェックできるようにするのが目的。
(defun check
  (task ;; taskに定義されているcoordsやwrenchのうち、最初の要素を利用してチェック
  &key
   (pre-process-func)
   (post-process-func)
   &allow-other-keys
   )
  ;; Visualize
  (let* (;; 可視化用オブジェクト。(send-all *task-arrow-list* :worldcoords)などはしない。
         (task-arrow-list
          (remove nil
                  (mapcar #'(lambda (coords wrench)
                              (if (equal (subseq wrench 0 3) (float-vector 0 0 0))
                                  nil
                                (make-single-arrow :length 200
                                                   :pos (send coords :worldpos)
                                                   :axis (subseq wrench 0 3)
                                                   :color #f(1 0.3 1))))
                          (list (car (send task :coords-list)))
                          (list (car (send task :wrench-list)))
                          )))
         (additional-draw-objects
          (append *room610-model-list*
                  (list (send (car (send task :attachable-plane-list)) :object))
                  task-arrow-list)))
    (unless (boundp '*irtviewer*)
      (make-irtviewer))
    (send *irtviewer* :objects additional-draw-objects)
    (set-viewpoint "room610")
    (send *irtviewer* :draw-objects)
    (set-room610-viewpoint))

  ;; Optimization
  (setq *robot* (yamaguchi-arm))
  (send *robot* :newcoords (make-coords))
  (send *robot* :leg-init :pos 50) ;; TODO: ここを現実のサイズに固定して実験する
  (let* ((move-target (list (send *robot* :end-coords)
                            (send *robot* :get :rigid-leg0-coords)
                            (send *robot* :get :rigid-leg1-coords)
                            (send *robot* :get :rigid-leg2-coords)
                            (send *robot* :get :rigid-leg3-coords)
                            (send *robot* :get :vacuum-pad-coords)
                            (send *robot* :get :vacuum-pad-normal-coords)
                            ))
         (limb-size (1- (length move-target)))
         target-coords
         (task-coords (car (send task :coords-list)))
         (external-wrench (car (send task :wrench-list)))
         (contact-constraint
          (list
           ;; TODO: 支持多角形を現実のサイズに固定して実験する
           (instance rigid-leg-contact-constraint :init :name "rigid-leg 0" :size 2.5)
           (instance rigid-leg-contact-constraint :init :name "rigid-leg 1" :size 2.5)
           (instance rigid-leg-contact-constraint :init :name "rigid-leg 2" :size 2.5)
           (instance rigid-leg-contact-constraint :init :name "rigid-leg 3" :size 2.5)
           (instance vacuum-pad-contact-constraint :init :name "vacuum-pad-negative")
           (instance rigid-leg-contact-constraint :init :name "vacuum-pad-normal" :size 40 :shape :octagon)
           ))
         (translation-axis (append (list t) (make-list limb-size :initial-element nil)))
         (rotation-axis (append (list nil) (make-list limb-size :initial-element nil)))
         (collision-avoidance-link-pair (list
                                         (list (elt (send *robot* :links) 0) (elt (send *robot* :links) 2))
                                         (list (elt (send *robot* :links) 0) (elt (send *robot* :links) 3))
                                         (list (elt (send *robot* :links) 0) (car (last (send *robot* :links))))
                                         ))
         sqp-opt
         )
    (let (task-face)
      (dotimes (j limb-size)
        (setq task-face (append task-face (list (send (car (send task :attachable-plane-list))
                                                      :face :margin 0
                                                      :name (format nil "~A" j)
                                                      )))))
      (setq target-coords (append (list (send task-coords :copy-worldcoords)) task-face)))
    (setq sqp-opt (send *robot*
                        :inverse-kinematics-statics-optmotiongen
                        target-coords
                        :move-target move-target
                        :contact-target-coords (cdr target-coords)
                        :contact-move-target (cdr move-target)
                        :contact-constraint contact-constraint
                        ;; メモ：external-wrenchは、手先が発揮しているレンチではなく、手先が受けているレンチ
                        :external-wrench-list (list external-wrench)
                        :external-coords-list (list (send *robot* :end-coords))
                        :translation-axis translation-axis
                        :rotation-axis rotation-axis
                        :collision-avoidance-link-pair collision-avoidance-link-pair
                        :collision-distance-limit -10 ;; 多少のめり込みを許容する？と、IKの成功率が格段に上がる[mm]
                        :debug-view (list :additional-draw-objects (send *irtviewer* :objects))
                        :root-virtual-mode :6dof
                        :optimize-torque? nil
                        :pre-process-func pre-process-func
                        :post-process-func post-process-func))

    ;; Print result
    (let* ((config-task (send sqp-opt :config-task))
           (contact-attention-coords-list (send config-task :contact-attention-coords-list))
           (contact-target-coords-list (send config-task :contact-target-coords-list)))
      (format t "~%===== Result (in world coords) =====~%")
      (format t "task: ~A~%" (send task-coords :name))
      (format t "angle vector~%")
      (format t " ~A~%" (send (send config-task :robot-env) :robot :angle-vector))
      ;; Look at vacuum base shape
      (let ((robot (copy-object *robot*))
            (v (send-lexpr (instantiate x::irtviewer) :create nil))
            )
        (send robot :newcoords (make-coords))
        (send robot :angle-vector
              (send (send config-task :robot-env) :robot :angle-vector))
        (send v :draw-origin nil) ;; 原点のRGBを消す
        (send v :objects (list robot))
        (send v :viewer :viewing :look #f(0 0 -1000) #f(0.0 0.0 0.0) #f(0.0 1.0 0.0))
        (send v :draw-objects))
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
                  (format t "   At ~A, ~A~%"
                          (elt (send-all contact-attention-coords-list :name) j)
                          (elt (send config-task force-type) j))))
              (when (or (eq force-type :external-force-list) (eq force-type :external-moment-list))
                (dotimes (j (length (send config-task :external-coords-list)))
                  (format t "   At ~A, ~A~%"
                          (send (elt (send config-task :external-coords-list) j) :worldcoords)
                          (elt (send config-task force-type) j))
                  )))
          (format t "   nil~%")))
      (format t "~%")
      (format t "Status: ~A~%==================~%~%" (send sqp-opt :status)))
    sqp-opt)
    )
(warn "(check *faucet-task*)~%")
