(load "../inverse-kinematics-statics-wrapper.l")
(load "../discrete-kinematics-configuration-task.l")
(load "../sqp-msc-optimization.l")

(load "room610-model-list.l") ;; Set *room610-model-list*
(load "task.l") ;; Set *tasks*
(load "yamaguchi-arm.l") ;; Define (yamaguchi-arm) function
(load "contact-constrant-models.l")
(load "base-shape-constraint.l") ;; Add equality constraint for base shape

(defun a ()
  ;; 電子レンジ操作：一方向の力を耐えればいいので、4つの剛体脚が1つの場所に集まって直線状のベース形状が出力された
  ;; (main (list *microwave-task*))
  ;; 料理：異なる場所に力を発揮するには、直線形状ではモーメントに耐えられず、三角形ベース形状が出力された
  ;; (main (list *faucet-task* *IH-task*))
  ;; 拭き掃除：ベースの周囲全方向に力を発揮するため、吸着パッドを内部に含む四角形のベース形状が出力される
  (main (list *table-task* *fridge-task*))

  ;; 全タスク。主にタスク環境可視化用
  ;; (main (list
  ;;        *microwave-task*
  ;;        *faucet-task*
  ;;        *IH-task*
  ;;        *table-task*
  ;;        *fridge-task*
  ;;        ))
  )
(warn "(a)~%")

;; 研究で使うメイン関数。吸着ベースの設計パラメータを時不変変数として扱っているのが特徴。
(defun main
    (tasks
     &key
     (pre-process-func)
     (post-process-func)
     (visualize-callback-func)
     (debug-view t)
     &allow-other-keys
     )
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
                          (flatten (send-all tasks :coords-list))
                          (flatten (send-all tasks :wrench-list))
                          )))
         (attachable-plane-list (flatten (send-all tasks :attachable-plane-list)))
         (additional-draw-objects
          (append *room610-model-list*
                  (send-all attachable-plane-list :object)
                  ;; (send-all *attachable-plane-list* :normal-object) ;; For debug
                  task-arrow-list)))
    ;; 2. Visualize
    (when debug-view
      (unless (boundp '*irtviewer*)
        (make-irtviewer))
      (send *irtviewer* :objects additional-draw-objects)
      (set-viewpoint "room610")
      (send *irtviewer* :draw-objects))
    ;; 3. optimization
    (setq *robot* (yamaguchi-arm))
    ;; (send *robot* :leg-init :pos 50)
    (send *robot* :move-rigid-leg-to-original)
    ;; (send *robot* :fix-leg)
    (let* ((move-target (list (send *robot* :end-coords)
                              (send *robot* :get :rigid-leg0-coords)
                              (send *robot* :get :rigid-leg1-coords)
                              (send *robot* :get :rigid-leg2-coords)
                              (send *robot* :get :rigid-leg3-coords)
                              (send *robot* :get :vacuum-pad-coords)
                              (send *robot* :get :vacuum-pad-normal-coords)
                              ))
           (limb-size (1- (length move-target)))
           (task-coords-list (flatten (send-all tasks :coords-list)))
           (task-wrench-list (flatten (send-all tasks :wrench-list)))
           (task-target-coords (send-all task-coords-list :copy-worldcoords))
           (task-size (length task-coords-list))
           contact-target-coords-list
           contact-move-target-list
           (translation-axis-list (make-list task-size :initial-element
                                             (append (list t) (make-list limb-size :initial-element nil))))
           (rotation-axis-list (make-list task-size :initial-element
                                          (append (list nil) (make-list limb-size :initial-element nil))))
           (collision-avoidance-link-pair (list
                                             (list (elt (send *robot* :links) 0) (elt (send *robot* :links) 2))
                                             (list (elt (send *robot* :links) 0) (elt (send *robot* :links) 3))
                                             (list (elt (send *robot* :links) 0) (car (last (send *robot* :links))))
                                             ))
           sqp-opt target-coords-list move-target-list contact-constraint-list
           same-position-task-index temporary-invariant-joint-list adjacent-regular-scale-list
           ;; (external-coords-list (mapcar #'list task-coords-list))
           (external-coords-name-list (make-list (length task-wrench-list) :initial-element (list :end-coords)))
           (external-wrench-list (mapcar #'list task-wrench-list))
           )
      (dotimes (i task-size)
        ;; メモ：各平面に固有の名前を付けないと、inverse-kinematics-statics-wrapper.lの以下のコードで除去される
        ;; (remove-duplicates (flatten target-coords-list) :test #'equal)
        (let (task-face)
          (dotimes (j limb-size)
            (setq task-face (append task-face (list (send (elt attachable-plane-list i)
                                                          :face :margin 0
                                                          :name (format nil "~A,~A" i j)
                                                          )))))
          (setq target-coords-list
                (append target-coords-list
                        (list (append (list (elt task-target-coords i))
                                      task-face)))))
        (setq move-target-list (append move-target-list (list move-target)))
        (setq contact-constraint-list (append contact-constraint-list
                                              (list
                                               (list
                                                (instance rigid-leg-contact-constraint :init :name (format nil "rigid-leg 0, ~A" i) :size 2.5)
                                                (instance rigid-leg-contact-constraint :init :name (format nil "rigid-leg 1, ~A" i) :size 2.5)
                                                (instance rigid-leg-contact-constraint :init :name (format nil "rigid-leg 2, ~A" i) :size 2.5)
                                                (instance rigid-leg-contact-constraint :init :name (format nil "rigid-leg 3, ~A" i) :size 2.5)
                                                (instance vacuum-pad-contact-constraint :init :name (format nil "vacuum-pad-negative ~A" i))
                                                (instance rigid-leg-contact-constraint :init :name (format nil "vacuum-pad-normal ~A" i) :size 40 :shape :octagon)
                                                )
                                               )))
        )
      (setq contact-target-coords-list (mapcar #'cdr target-coords-list))
      (setq contact-move-target-list (mapcar #'cdr move-target-list))
      ;; same-position-task-indexが(list i j)のとき、i, i+1, j, j+1番目のタスクが同じ場所で実行されるという意味
      (dotimes (i (- (length task-coords-list) 1))
        (when (equal (send (elt task-coords-list i) :name)
                     (send (elt task-coords-list (+ i 1)) :name))
          (setq same-position-task-index (append same-position-task-index (list i)))))
      (dolist (j (list :yamaguchi_6axis_arm_with_vacuum_base-root-virtual-linear-joint-x
                       :yamaguchi_6axis_arm_with_vacuum_base-root-virtual-linear-joint-y
                       :yamaguchi_6axis_arm_with_vacuum_base-root-virtual-linear-joint-z
                       :yamaguchi_6axis_arm_with_vacuum_base-root-virtual-rotational-joint-x
                       :yamaguchi_6axis_arm_with_vacuum_base-root-virtual-rotational-joint-y
                       :yamaguchi_6axis_arm_with_vacuum_base-root-virtual-rotational-joint-z))
        (dolist (i same-position-task-index)
          (setq temporary-invariant-joint-list (append temporary-invariant-joint-list (list (list j i))))))
      (setq adjacent-regular-scale-list (make-list (- task-size 1) :initial-element 0))
      (dolist (l same-position-task-index)
        (setf (elt adjacent-regular-scale-list l) 5e-4))
      ;; 1回目は、運動学のみで解を出す。
      ;; 2回目は、この解を初期値にして、次に静力学のみで解を出す。
      ;; 3回目以降は、前回の最適化計算の値を初期値にしてbase-size-task-scaleを大きくしていくことで、ベースサイズを小さくする。静力学の範囲で解は出ているので、kin-scaleとeom-scaleは小さくしていく。
      ;; この手順を踏むことで、運動学の問題、静力学の問題、ベースサイズ最小化問題を別々に解けるので、安定して解を出せるはず。
      (let (
            (only-kinematics?-list (list t nil nil nil nil nil nil))
            (kin-scale-list (list 5.0 5.0 0.01 0.01 0.01 0.01 0.01))
            ;; (kin-scale-list (list 5.0 5.0 1.0 1.0 1.0 1.0 1.0))
            (eom-scale-list (list 0.0 1.0 0.01 0.01 0.01 0.01 0.01))
            ;; (base-size-task-scale-list (list 0 0 (* 0.1 1e-4) (* 0.5 1e-4) (* 0.7 1e-4) (* 1.0 1e-4) (* 3.0 1e-4)))
            ;; (base-size-task-scale-list (list 0 0 (* 0.1 1e-4) (* 0.5 1e-4) (* 0.7 1e-4) (* 1.0 1e-4) (* 3.0 1e-4)))
            (base-size-task-scale-list (list 0 0 (* 0.01 1e-4) (* 0.03 1e-4) (* 0.05 1e-4) (* 0.1 1e-4) (* 0.3 1e-4)))
            prev-config-task)
        (unless (and (eq (length only-kinematics?-list) (length kin-scale-list))
                     (eq (length only-kinematics?-list) (length eom-scale-list))
                     (eq (length only-kinematics?-list) (length base-size-task-scale-list))
                     )
          (print "Iteration parameters does not correspond.")
          (exit))
        (dotimes (i (length only-kinematics?-list))
          (setq *base-size-task-scale* (elt base-size-task-scale-list i))
          (format t "Start optimization with the following parameters~%")
          (format t "only-kinematics? ~A~%" (elt only-kinematics?-list i))
          (format t "kin-scale ~A~%" (elt kin-scale-list i))
          (format t "eom-scale ~A~%" (elt eom-scale-list i))
          (format t "*base-size-task-scale* ~A~%" *base-size-task-scale*)
          ;; 前回のsqpの結果を初期値にして、新しい最適化計算を行う
          (when sqp-opt
            (setq prev-config-task (send sqp-opt :config-task)))
          ;; 静力学を考えない時は、剛体脚の位置を固定にしておく
          ;; ベースサイズをタスク関数に入れない時は、剛体脚の位置を固定にしておく
          (if (or (eq *base-size-task-scale* 0) (eq *base-size-task-scale* 0.0))
              (send *robot* :fix-leg)
            (send *robot* :unfix-leg))
          (setq sqp-opt (send *robot*
                              :inverse-kinematics-statics-trajectory-optmotiongen
                              target-coords-list
                              :move-target-list move-target-list
                              :contact-target-coords-list contact-target-coords-list
                              :contact-move-target-list contact-move-target-list
                              :contact-constraint-list contact-constraint-list
                              ;; メモ：external-wrench-listは、手先が発揮しているレンチではなく、手先が受けているレンチ
                              :external-wrench-list external-wrench-list
                              :external-coords-name-list external-coords-name-list
                              :translation-axis-list translation-axis-list
                              :rotation-axis-list rotation-axis-list
                              :collision-avoidance-link-pair collision-avoidance-link-pair
                              :collision-distance-limit -0.5
                              :invariant-joint-list (send *robot* :invariant-joint-list)
                              :temporary-invariant-joint-list temporary-invariant-joint-list
                              :adjacent-regular-scale-list adjacent-regular-scale-list
                              :root-virtual-mode :6dof
                              :optimize-torque? nil
                              ;; メモ：kin-scaleやeom-scale値がデフォルトの1.0のままだと、収束しづらいことがある。
                              :kin-scale (elt kin-scale-list i)
                              :eom-scale (elt eom-scale-list i)
                              ;; :stop 50
                              ;; :min-loop 30
                              ;; :stop 100
                              ;; :min-loop 100
                              :debug-view (when debug-view (list :additional-draw-objects additional-draw-objects))
                              :only-kinematics? (elt only-kinematics?-list i)
                              :initial-config-task prev-config-task
                              :pre-process-func pre-process-func
                              :post-process-func post-process-func))
          (send *robot* :unfix-leg)
          ;; (visualize-trajectory-result ret :visualize-callback-func visualize-callback-func)

          ;; 4. Print result
          (let* ((traj-config-task (send sqp-opt :config-task))
                 (instant-config-task-list (send traj-config-task :instant-config-task-list)))
            (format t "~%===== Result (in world coords) =====~%")
            (dotimes (i (length instant-config-task-list))
              (format t "task ~A: ~A~%" i (send (elt task-coords-list i) :name))
              (let* ((config-task (elt instant-config-task-list i))
                     (contact-attention-coords-list (send config-task :contact-attention-coords-list))
                     (contact-target-coords-list (send config-task :contact-target-coords-list)))
                (format t "angle vector~%")
                (format t " ~A~%" (send (send config-task :robot-env) :robot :angle-vector))
                ;; Look at vacuum base shape
                (when (and debug-view (eq i 0))
                  (let ((robot (copy-object *robot*))
                        (v (send-lexpr (instantiate x::irtviewer) :create nil))
                        )
                    (send robot :newcoords (make-coords))
                    (send robot :angle-vector
                          (send (send config-task :robot-env) :robot :angle-vector))
                    (send v :draw-origin nil) ;; 原点のRGBを消す
                    (send v :objects (list robot))
                    (send v :viewer :viewing :look #f(0 0 -1000) #f(0.0 0.0 0.0) #f(0.0 1.0 0.0))
                    (send v :draw-objects)))
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
                            (format t "   At ~A, ~A~%"
                                    (elt (send-all contact-attention-coords-list :name) j)
                                    (elt (send config-task force-type) j))))
                        (when (or (eq force-type :external-force-list) (eq force-type :external-moment-list))
                          (dotimes (j (length (send config-task :external-coords-list)))
                            (format t "   At ~A, ~A~%"
                                    (send (elt (send config-task :external-coords-list) j) :worldcoords)
                                    (elt (send config-task force-type) j))
                            )))
                    (format t "   nil~%"))))
              (format t "~%")))
          (format t "only-kinematics? ~A~%" (elt only-kinematics?-list i))
          (format t "kin-scale ~A~%" (elt kin-scale-list i))
          (format t "eom-scale ~A~%" (elt eom-scale-list i))
          (format t "*base-size-task-scale* ~A~%" *base-size-task-scale*)
          (format t "Status: ~A~%==================~%~%" (send sqp-opt :status))
          (when (not (eq (send sqp-opt :status) :success))
            (format t "Failed to optimize~%")
            (return-from main sqp-opt))))
      sqp-opt
      )))
