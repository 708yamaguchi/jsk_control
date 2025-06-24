(load "../inverse-kinematics-statics-wrapper.l")
(load "../discrete-kinematics-configuration-task.l")
(load "../sqp-msc-optimization.l")

(load "room610-model-list.l") ;; Set *room610-model-list*
(load "attachable-plane.l")
(load "yamaguchi-arm.l") ;; Define (yamaguchi-arm) function
(load "contact-constrant-models.l")
(load "base-shape-constraint.l") ;; Add equality constraint for base shape

;; 1. Prepare task environment
;; If you change task configuration, change only this section.
;; The following variables are the same length and correspond in order
;; 全く同じ位置の平面は与えられなかったので、同じ場所で複数タスクを行う場合は微小量だけ動かした平面を利用
(setq *attachable-plane-list*
      (list
       ;; faucet
       (instance attachable-plane :init
                 :offset 900
                 :min-angle-list (list 7800 -250)
                 :max-angle-list (list 8300 600)
                 :normal :z+)
       ;; (instance attachable-plane :init
       ;;           :offset 900.001
       ;;           :min-angle-list (list 7800 -250)
       ;;           :max-angle-list (list 8300 600)
       ;;           :normal :z+)
       ;; (instance attachable-plane :init
       ;;           :offset 900.002
       ;;           :min-angle-list (list 7800 -250)
       ;;           :max-angle-list (list 8300 600)
       ;;           :normal :z+)
       ;; IH Z軸方向で、キッチンのちょっとした側まで平面を伸ばすと成功率が上がる
       (instance attachable-plane :init
                 :offset 8500
                 :min-angle-list (list -1250 800)
                 :max-angle-list (list 1600 2000)
                 :normal :x-)
       (instance attachable-plane :init
                 :offset 8500.001
                 :min-angle-list (list -1250 800)
                 :max-angle-list (list 1600 2000)
                 :normal :x-)
       ;; table
       (instance attachable-plane :init
                 :offset 720
                 :min-angle-list (list 4300 -700)
                 :max-angle-list (list 5000 150)
                 :normal :z+)
       (instance attachable-plane :init
                 :offset 720.001
                 :min-angle-list (list 4300 -700)
                 :max-angle-list (list 5000 150)
                 :normal :z+)
       (instance attachable-plane :init
                 :offset 720.002
                 :min-angle-list (list 4300 -700)
                 :max-angle-list (list 5000 150)
                 :normal :z+)
       ;; microwave
       (instance attachable-plane :init
                 :offset 1200
                 :min-angle-list (list 6050 1850)
                 :max-angle-list (list 6450 2200)
                 :normal :z+)
       ;; fridge
       (instance attachable-plane :init
                 :offset 1850
                 :min-angle-list (list 6720 2010)
                 :max-angle-list (list 7300 2520)
                 :normal :z+)
       ;; (instance attachable-plane :init
       ;;           :offset 1850.001
       ;;           :min-angle-list (list 6720 2010)
       ;;           :max-angle-list (list 7300 2520)
       ;;           :normal :z+)
       (instance attachable-plane :init
                 :offset 1850.002
                 :min-angle-list (list 6720 2010)
                 :max-angle-list (list 7300 2520)
                 :normal :z+)
       ;; (instance attachable-plane :init
       ;;           :offset 1850.003
       ;;           :min-angle-list (list 6720 2010)
       ;;           :max-angle-list (list 7300 2520)
       ;;           :normal :z+)
       (instance attachable-plane :init
                 :offset 1850.004
                 :min-angle-list (list 6720 2010)
                 :max-angle-list (list 7300 2520)
                 :normal :z+)
       ))
;; 注意：同じ場所で実行するタスクは、リストの隣り合う要素として記述すること
(setq *task-coords-list*
      (list
       (make-cascoords :pos (float-vector 8200 700 1000) :rpy (list (deg2rad 60) 0 0)
                       :name "faucet")
       ;; (make-cascoords :pos (float-vector 8300 700 1100) :rpy (list (deg2rad 60) 0 0)
       ;;                 :name "faucet")
       ;; (make-cascoords :pos (float-vector 7800 600 1000) :rpy (list (deg2rad 60) 0 0)
       ;;                 :name "faucet")
       (make-cascoords :pos (float-vector 8200 -600 1000) :rpy (list 0 0 0)
                       :name "IH")
       (make-cascoords :pos (float-vector 8200 -500 1000) :rpy (list 0 0 0)
                       :name "IH")
       (make-cascoords :pos (float-vector 4650 0 740) :rpy (list 0 0 0)
                       :name "table")
       (make-cascoords :pos (float-vector 4500 -150 740) :rpy (list 0 0 0)
                       :name "table")
       (make-cascoords :pos (float-vector 4650 -300 740) :rpy (list 0 0 0)
                       :name "table")
       (make-cascoords :pos (float-vector 6200 1800 1200) :rpy (list 0 0 0)
                       :name "microwave")
       (make-cascoords :pos (float-vector 6900 2400 1850) :rpy (list 0 0 0)
                       :name "fridge")
       ;; (make-cascoords :pos (float-vector 6850 2325 1850) :rpy (list 0 0 0)
       ;;                 :name "fridge")
       (make-cascoords :pos (float-vector 6800 2250 1850) :rpy (list 0 0 0)
                       :name "fridge")
       ;; (make-cascoords :pos (float-vector 6850 2175 1850) :rpy (list 0 0 0)
       ;;                 :name "fridge")
       (make-cascoords :pos (float-vector 6900 2100 1850) :rpy (list 0 0 0)
                       :name "fridge")
       ))
;; TODO: タスクに合っている力の制約を入れる。タスクの場所も増やす
;; wrenchを少し変えるだけでも最適化の成否が変わるのが難しい
(setq *task-wrench-list*
      (list
       ;; faucet
       (float-vector 0 0 -1 0 0 0)
       ;; (float-vector 0 0 -1 0 0 0)
       ;; (float-vector 0 0 -1 0 0 0)
       ;; IH
       (float-vector 0 0 -1 0 0 0)
       (float-vector 0 0 -1 0 0 0)
       ;; table
       (float-vector 0 0 -1 0 0 0)
       (float-vector 0 0 -1 0 0 0)
       (float-vector 0 0 -1 0 0 0)
       ;; microwave
       (float-vector 0 0 -1 0 0 0)
       ;; fridge
       (float-vector 0 0 1 0 0 0)
       ;; (float-vector 0 0 1 0 0 0)
       (float-vector 0 0 1 0 0 0)
       ;; (float-vector 0 0 1 0 0 0)
       (float-vector 0 0 1 0 0 0)
       ))
;; 可視化用オブジェクト。(send-all *task-arrow-list* :worldcoords)などはしない。
(setq *task-arrow-list*
      (mapcar #'(lambda (coords wrench)
                  (if (equal (subseq wrench 0 3) (float-vector 0 0 0))
                      nil
                    (make-single-arrow :length 200
                                       :pos (send coords :worldpos)
                                       :axis (subseq wrench 0 3)
                                       :color #f(1 0.3 1))))
              *task-coords-list* *task-wrench-list*))
(setq *task-arrow-list* (remove nil *task-arrow-list*))

(unless (and (eq (length *attachable-plane-list*) (length *task-coords-list*))
             (eq (length *task-coords-list*) (length *task-wrench-list*)))
  (print "Task definition is wrong.")
  (exit))


;; TODO: Create edge case test functions

;;このテストから、 ベースサイズタスクの重みが小さいほど、大きな形状のベースが出力されることが分かる。
(defun test-low-base-size-task-scale ()
  (main :base-size-task-scale (* 0.01 *default-base-size-task-scale*))
  )

;; 研究で使うメイン関数。吸着ベースの設計パラメータを時不変変数として扱っているのが特徴。
(defun main
    (&key
     (pre-process-func)
     (post-process-func)
     (visualize-callback-func)
     (base-size-task-scale)
     (debug-view t)
     &allow-other-keys
     )
  (let* ((additional-draw-objects
          (append *room610-model-list*
                  (send-all *attachable-plane-list* :object)
                  ;; (send-all *attachable-plane-list* :normal-object) ;; For debug
                  *task-arrow-list*)))
    ;; 2. Visualize
    (when debug-view
      (unless (boundp '*irtviewer*)
        (make-irtviewer))
      (send *irtviewer* :objects additional-draw-objects)
      (set-viewpoint "room610")
      (send *irtviewer* :draw-objects))
    ;; 3. optimization
    (setq *robot* (yamaguchi-arm))
    (send *robot* :leg-init :pos 50)
    (if base-size-task-scale
        (setq *base-size-task-scale* base-size-task-scale) ;; overwrite
      (setq *base-size-task-scale* *default-base-size-task-scale*)) ;; reset
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
           (external-coords-list (mapcar #'list *task-coords-list*))
           (external-wrench-list (mapcar #'list *task-wrench-list*))
           )
      (dotimes (i task-size)
        ;; メモ：各平面に固有の名前を付けないと、inverse-kinematics-statics-wrapper.lの以下のコードで除去される
        ;; (remove-duplicates (flatten target-coords-list) :test #'equal)
        (let (task-face)
          (dotimes (j limb-size)
            (setq task-face (append task-face (list (send (elt *attachable-plane-list* i)
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
                                                (instance rigid-leg-contact-constraint :init :name (format nil "rigid-leg 0, ~A" i))
                                                (instance rigid-leg-contact-constraint :init :name (format nil "rigid-leg 1, ~A" i))
                                                (instance rigid-leg-contact-constraint :init :name (format nil "rigid-leg 2, ~A" i))
                                                (instance rigid-leg-contact-constraint :init :name (format nil "rigid-leg 3, ~A" i))
                                                (instance vacuum-pad-contact-constraint :init :name (format nil "vacuum-pad ~A" i))
                                                )
                                               )))
        )
      (setq contact-target-coords-list (mapcar #'cdr target-coords-list))
      (setq contact-move-target-list (mapcar #'cdr move-target-list))
      ;; same-position-task-indexが(list i j)のとき、i, i+1, j, j+1番目のタスクが同じ場所で実行されるという意味
      (dotimes (i (- (length *task-coords-list*) 1))
        (when (equal (send (elt *task-coords-list* i) :name)
                     (send (elt *task-coords-list* (+ i 1)) :name))
          (setq same-position-task-index (append same-position-task-index (list i)))))
      (dolist (j (list :yamaguchi_6axis_arm_with_vacuum_base-root-virtual-linear-joint-x
                       :yamaguchi_6axis_arm_with_vacuum_base-root-virtual-linear-joint-y
                       :yamaguchi_6axis_arm_with_vacuum_base-root-virtual-rotational-joint-x
                       :yamaguchi_6axis_arm_with_vacuum_base-root-virtual-rotational-joint-y
                       :yamaguchi_6axis_arm_with_vacuum_base-root-virtual-rotational-joint-z))
        (dolist (i same-position-task-index)
          (setq temporary-invariant-joint-list (append temporary-invariant-joint-list (list (list j i))))))
      (setq adjacent-regular-scale-list (make-list (- task-size 1) :initial-element 0))
      (dolist (l same-position-task-index)
        (setf (elt adjacent-regular-scale-list l) 5e-4))
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
                          :temporary-invariant-joint-list temporary-invariant-joint-list
                          :adjacent-regular-scale-list adjacent-regular-scale-list
                          :root-virtual-mode :6dof
                          :optimize-torque? nil
                          :min-loop 10
                          ;; :stop 50
                          ;; :min-loop 30
                          ;; :stop 100
                          ;; :min-loop 100
                          :debug-view (when debug-view (list :additional-draw-objects additional-draw-objects))
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
            ;; Look at vacuum base shape
            (when (and debug-view (eq i 0))
              (let ((robot (copy-object *robot*))
                    (v (send-lexpr (instantiate x::irtviewer) :create nil)))
                (send robot :newcoords (make-coords))
                (send robot :angle-vector
                      (send (send config-task :robot-env) :robot :angle-vector))
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
      (format t "Status: ~A~%==================~%~%" (send sqp-opt :status))
      sqp-opt)))
(warn "(main)~%")
