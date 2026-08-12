;;
;; main-multi-robot.l - Multi-robot task runner for Yamaguchi-arm and R8-arm
;;

(load "../inverse-kinematics-statics-wrapper.l")
(load "../discrete-kinematics-configuration-task.l")
(load "../sqp-msc-optimization.l")

(load "room610-model-list.l")
(load "task.l")
(load "util.l")
(load "yamaguchi-arm.l")
(load "r8-arm.l") ;; R8用のモデル定義をロード
(load "contact-constrant-models.l")

(defun create-robot (robot-type &key (pad-diameter 60))
  "指定したロボット種別のインスタンスを生成する"
  (case robot-type
    (:yamaguchi
     (yamaguchi-arm :pad-diameter pad-diameter :transparency 0.4))
    (:r8
     ;; 使用する手の左右を切り替えられる。
     (let ((robot (instance r8-arm-robot :init :arm :rarm)))
       (send robot :startup-pose)
       robot))
    (t (error "Unsupported robot-type: ~A" robot-type))))

(defun run
    (&key
     (task :faucet) ;; :faucet, :faucet-wall, :ih, :wipe-ih-wall, :microwave, :table, :fridge
     (robot-type :yamaguchi) ;; :yamaguchi または :r8
     (pad-diameter 60)
     (debug-view t)
     (stop-num 50)
     (stop nil)
     (random-angle-vector-seed :auto))
  "タスクをYamaguchi-armまたはR8で実行する"
  (let* (task-obj viewpoint seed-val robot tasks args sqp-opt start-time
         (max-stop (or stop stop-num)))

    ;; 1. タスク種別と視点・初期シードの解決 (main.lのコメントに拠るシード値)
    (cond
     ((or (eq task :faucet) (equal task "faucet"))
      (setq task-obj (if (eq robot-type :r8) *r8-faucet-task* *faucet-task*))
      (setq viewpoint "faucet")
      (setq seed-val (if (eq random-angle-vector-seed :auto) (list 860217820 1488608567) random-angle-vector-seed)))
     ((or (eq task :faucet-wall) (equal task "faucet-wall"))
      (setq task-obj (if (eq robot-type :r8) *r8-faucet-task* *faucet-wall-task*))
      (setq viewpoint "faucet")
      (setq seed-val (if (eq random-angle-vector-seed :auto) (list 1777581173 1619544386) random-angle-vector-seed)))
     ((or (eq task :ih) (eq task :IH) (equal task "IH") (equal task "ih"))
      (setq task-obj (if (eq robot-type :r8) *r8-ih-task* *IH-task*))
      (setq viewpoint "IH")
      (setq seed-val (if (eq random-angle-vector-seed :auto) (list 742790772 227240069) random-angle-vector-seed)))
     ((or (eq task :wipe-ih-wall) (equal task "wipe-ih-wall"))
      (setq task-obj (if (eq robot-type :r8) *r8-wipe-ih-wall-task* *wipe-ih-wall-task*))
      (setq viewpoint "IH")
      (setq seed-val (if (eq random-angle-vector-seed :auto) (list 860217820 1488608567) random-angle-vector-seed)))
     ((or (eq task :microwave) (equal task "microwave"))
      (setq task-obj (if (eq robot-type :r8) *r8-microwave-task* *microwave-task*))
      (setq viewpoint "microwave")
      (setq seed-val (if (eq random-angle-vector-seed :auto) (list 1654553012 786543381) random-angle-vector-seed)))
     ((or (eq task :table) (equal task "table"))
      (setq task-obj (if (eq robot-type :r8) *r8-table-task* *table-task*))
      (setq viewpoint "table")
      (setq seed-val (if (eq random-angle-vector-seed :auto) nil random-angle-vector-seed)))
     ((or (eq task :fridge) (equal task "fridge"))
      (setq task-obj (if (eq robot-type :r8) *r8-fridge-task* *fridge-task*))
      (setq viewpoint "fridge")
      (setq seed-val (if (eq random-angle-vector-seed :auto) nil random-angle-vector-seed)))
     ((derivedp task task)
      (setq task-obj task)
      (setq viewpoint (send task :name))
      (setq seed-val (if (eq random-angle-vector-seed :auto) nil random-angle-vector-seed)))
     (t
      (error "Unsupported task: ~A" task)))

    (setq robot (create-robot robot-type :pad-diameter pad-diameter))
    (setq tasks (list task-obj))

    ;; 2. ロボットごとの最適化実行
    (case robot-type
      (:r8
       ;; R8: 運動学・ベース移動を含めて解く
       (send robot :newcoords (make-coords))
       (when debug-view
         (visualize-tasks tasks)
         (when viewpoint
           (set-viewpoint viewpoint)))
       (setq start-time (car (unix:ptimes)))
       (setq args (get-r8-args robot tasks))
       (setq sqp-opt
             (send robot
                   :inverse-kinematics-trajectory-optmotiongen
                   (gethash :target-coords-list args)
                   :move-target-list (gethash :move-target-list args)
                   :translation-axis-list (gethash :translation-axis-list args)
                   :rotation-axis-list (gethash :rotation-axis-list args)
                   :collision-avoidance-link-pair (gethash :collision-avoidance-link-pair args)
                   :collision-distance-limit 5
                   :root-virtual-joint-invariant? t
                   :thre 3.0
                   :stop max-stop
                   :root-virtual-mode :6dof
                   :dissoc-root-virtual nil
                   :debug-view (when debug-view (list :additional-draw-objects (send *irtviewer* :objects)))))
       (format t "Optimization 1: elapsed time ~A [s]~%" (* 0.01 (- (car (unix:ptimes)) start-time)))
       (if (eq (send sqp-opt :status) :success)
           (format t "Optimization succeeded.~%")
         (format t "Optimization failed.~%")))

      (:yamaguchi
       ;; Yamaguchi-arm: 静力学・接触制約・ベース設計パラメータを含めて段階的に解く
       (let* ((kin-scale-list (list 0.4 0.4 0.4 0.4 0.4 0.4))
              (base-size-scale-list (list 0 (* 4.0 1e-4) (* 6.0 1e-4) (* 8.0 1e-4) (* 10.0 1e-4) (* 10.0 1e-4)))
              (fix-variant-joint-list (list nil nil nil nil nil nil))
              (loop-num-min 30) (loop-num-max max-stop))
         (send robot :fix-gripper)
         (if seed-val
             (progn
               (setq *random-state*
                     (integer-vector (elt seed-val 0)
                                     (elt seed-val 1)))
               (send robot :random-angle-vector))
           (send robot :move-rigid-leg-to-original))
         (send robot :newcoords (make-coords))
         (when debug-view
           (visualize-tasks tasks)
           (when viewpoint
             (set-viewpoint viewpoint)))
         (setq start-time (car (unix:ptimes)))
         (setq args (get-args robot tasks))

         ;; 1回目の最適化
         (setq sqp-opt
               (send robot
                     :inverse-kinematics-statics-trajectory-optmotiongen
                     (gethash :target-coords-list args)
                     :move-target-list (gethash :move-target-list args)
                     :contact-target-coords-list (gethash :contact-target-coords-list args)
                     :contact-move-target-list (gethash :contact-move-target-list args)
                     :contact-constraint-list (gethash :contact-constraint-list args)
                     :external-wrench-list (gethash :external-wrench-list args)
                     :external-coords-list (gethash :external-coords-list args)
                     :translation-axis-list (gethash :translation-axis-list args)
                     :rotation-axis-list (gethash :rotation-axis-list args)
                     :collision-avoidance-link-pair (gethash :collision-avoidance-link-pair args)
                     :collision-distance-limit 5
                     :invariant-joint-list (send robot :invariant-joint-list)
                     :temporary-invariant-joint-list (gethash :temporary-invariant-joint-list args)
                     :adjacent-regular-scale-list (gethash :adjacent-regular-scale-list args)
                     :only-kinematics? nil
                     :kin-scale (car kin-scale-list)
                     :base-size-scale (car base-size-scale-list)
                     :initial-sqp-opt nil
                     :dissoc-root-virtual nil
                     :thre 3.0
                     :debug-view (when debug-view (list :additional-draw-objects (send *irtviewer* :objects)))
                     :revert-if-fail nil
                     :root-virtual-mode :6dof
                     :min-loop loop-num-min
                     :stop loop-num-max
                     :optimize-torque? nil))
         (format t "Optimization 1: elapsed time ~A [s]~%" (* 0.01 (- (car (unix:ptimes)) start-time)))
         (print-sqp-result sqp-opt args :debug-view nil)

         ;; 2回目以降の段階的最適化
         (dotimes (i (- (length kin-scale-list) 1))
           (let* ((config-task-list (send (send sqp-opt :config-task) :instant-config-task-list)))
             (send-all config-task-list :kin-scale (elt kin-scale-list (+ i 1)))
             (send-all config-task-list :base-size-scale (elt base-size-scale-list (+ i 1)))
             (let ((robot-list (send-all (send-all config-task-list :robot-env) :robot)))
               (send-all config-task-list :phi-max-vector :update? t)
               (send-all config-task-list :phi-min-vector :update? t))
             (if (elt fix-variant-joint-list (+ i 1))
                 (let* ((robot-env-list (send-all config-task-list :robot-env))
                        (arm-joint-list (send-all (send-all robot-env-list :robot) :variant-joint-list))
                        (root-virtual-joint-list (send-all robot-env-list :root-virtual-joint-list))
                        (orig-max-angle-list (mapcar #'(lambda (j) (send-all j :max-angle)) arm-joint-list))
                        (orig-min-angle-list (mapcar #'(lambda (j) (send-all j :min-angle)) arm-joint-list)))
                   (mapcar #'(lambda (jl) (mapcar #'(lambda (j) (send j :max-angle (send j :joint-angle))) jl)) arm-joint-list)
                   (mapcar #'(lambda (jl) (mapcar #'(lambda (j) (send j :min-angle (send j :joint-angle))) jl)) arm-joint-list)
                   (mapcar #'(lambda (jl) (mapcar #'(lambda (j) (send j :max-angle (send j :joint-angle))) jl)) root-virtual-joint-list)
                   (mapcar #'(lambda (jl) (mapcar #'(lambda (j) (send j :min-angle (send j :joint-angle))) jl)) root-virtual-joint-list)
                   (send-all config-task-list :theta-max-vector :update? t)
                   (send-all config-task-list :theta-min-vector :update? t)
                   (send sqp-opt :optimize :loop-num loop-num-max :loop-num-min loop-num-min)
                   (mapcar #'(lambda (jl max-angle-list) (mapcar #'(lambda (j m) (send j :max-angle m)) jl max-angle-list))
                           arm-joint-list orig-max-angle-list)
                   (mapcar #'(lambda (jl min-angle-list) (mapcar #'(lambda (j m) (send j :min-angle m)) jl min-angle-list))
                           arm-joint-list orig-min-angle-list)
                   (mapcar #'(lambda (jl) (mapcar #'(lambda (j) (send j :max-angle 1e10)) jl)) root-virtual-joint-list)
                   (mapcar #'(lambda (jl) (mapcar #'(lambda (j) (send j :min-angle -1e10)) jl)) root-virtual-joint-list)
                   (send-all config-task-list :theta-max-vector :update? t)
                   (send-all config-task-list :theta-min-vector :update? t))
               (send sqp-opt :optimize :loop-num loop-num-max :loop-num-min loop-num-min)))
           (format t "Optimization ~A: elapsed time ~A [s]~%" (+ i 2) (* 0.01 (- (car (unix:ptimes)) start-time)))
           (print-sqp-result sqp-opt args :debug-view nil))

         (if (eq (send sqp-opt :status) :success)
             (format t "Optimization succeeded.~%")
           (format t "Optimization failed.~%")))))

    ;; 仮想ルートジョイントの削除
    (when sqp-opt
      (send-all (send-all (send (send sqp-opt :config-task) :instant-config-task-list) :robot-env)
                :dissoc-root-virtual))

    sqp-opt))

(warn "(run :task :faucet :robot-type :yamaguchi)      ;; Yamaguchi-armでfaucetタスク実行~%")
(warn "(run :task :faucet-wall :robot-type :yamaguchi) ;; Yamaguchi-armでfaucet-wallタスク実行~%")
(warn "(run :task :ih :robot-type :yamaguchi)          ;; Yamaguchi-armでIHタスク実行 (*IH-task*)~%")
(warn "(run :task :wipe-ih-wall :robot-type :yamaguchi);; Yamaguchi-armでwipe-ih-wallタスク実行 (*wipe-ih-wall-task*)~%")
(warn "(run :task :microwave :robot-type :yamaguchi)   ;; Yamaguchi-armでmicrowaveタスク実行~%")
(warn "(run :task :faucet :robot-type :r8)             ;; R8で実行~%")
