(load "../inverse-kinematics-statics-wrapper.l")
(load "../discrete-kinematics-configuration-task.l")
(load "../sqp-msc-optimization.l")

(load "room610-model-list.l") ;; Set *room610-model-list*
(load "task.l") ;; Set *tasks*
(load "util.l")
(load "yamaguchi-arm.l") ;; Define (yamaguchi-arm) function
(load "contact-constrant-models.l")


(defun check-single
    (task)
  "
ベースの形状を固定にして:inverse-kinematics-statics-optmotiongenを解いて、静力学的に解が出るかをチェックする
現実と同じ物理パラメータになっているかをチェックできるようにするのが目的。

引数のtaskに定義されているcoordsやwrenchのうち、最初の要素を利用してチェック
"
  (let* ((robot (yamaguchi-arm :pad-diameter 80))
         (args (get-args robot (list task)))
         sqp-opt)

    ;; Visualize
    (visualize-tasks (list task))

    ;; Robot settings
    (send robot :move-rigid-leg-to-original)
    (send robot :fix-leg) ;; 剛体脚の位置を固定する
    (send robot :newcoords (make-coords))

    ;; Optimization
    (setq sqp-opt (send robot
                        :inverse-kinematics-statics-optmotiongen
                        (car (gethash :target-coords-list args))
                        :move-target (car (gethash :move-target-list args))
                        :contact-target-coords (car (gethash :contact-target-coords-list args))
                        :contact-move-target (car (gethash :contact-move-target-list args))
                        :contact-constraint (car (gethash :contact-constraint-list args))
                        :external-wrench-list (car (gethash :external-wrench-list args))
                        :external-coords-list (car (gethash :external-coords-list args))
                        :translation-axis (car (gethash :translation-axis-list args))
                        :rotation-axis (car (gethash :rotation-axis-list args))
                        :collision-avoidance-link-pair (gethash :collision-avoidance-link-pair args)
                        :collision-distance-limit -10 ;; 多少のめり込みを許容する？と、IKの成功率が格段に上がる[mm]
                        :debug-view (list :additional-draw-objects (send *irtviewer* :objects))
                        :revert-if-fail nil
                        :root-virtual-mode :6dof
                        :stop 100
                        :optimize-torque? nil))

    ;; Print result
    (print-sqp-result sqp-opt (list (send task :name)) :dissoc-root-virtual nil)

    sqp-opt))
(warn "(check-single *faucet-task*) ;; 単体作業場所に対して固定された剛体脚配置を使って静力学の整合性をチェック~%")

(defun check-task
    (task &key (kin-scale 1.0))
  (let* ((robot (yamaguchi-arm :pad-diameter 80))
         (args (get-args robot (list task)))
         (loop-num-min 30) (loop-num-max 100) ;; iteration回数は、loop-num-min以上、loop-num-max以下
         (task-name-list (make-list (send task :size) :initial-element (send task :name)))
         sqp-opt)

    ;; Visualize
    (visualize-tasks (list task))

    ;; Robot setting
    (send robot :move-rigid-leg-to-original)
    (send robot :fix-leg)
    (send robot :newcoords (make-coords))

    (setq sqp-opt (send robot
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
                        :collision-distance-limit 0 ;; 多少のめり込みを許容する？と、IKの成功率が格段に上がる[mm]
                        ;; :invariant-joint-list (send robot :invariant-joint-list) ;; ベースの設計パラメータを固定
                        :temporary-invariant-joint-list (gethash :temporary-invariant-joint-list args) ;; ベース配置をタスクごとに固定
                        :adjacent-regular-scale-list (gethash :adjacent-regular-scale-list args)
                        :only-kinematics? nil
                        :kin-scale kin-scale
                        :base-size-scale 0 ;; invariant-joint-listがないので、そもそもbase taskがないが、分かりやすさのため0にする。
                        :dissoc-root-virtual nil
                        :thre 5.0 ;; 手先の許容位置誤差
                        :debug-view (list :additional-draw-objects (send *irtviewer* :objects))
                        :revert-if-fail nil
                        :root-virtual-mode :6dof
                        :min-loop loop-num-min
                        :stop loop-num-max
                        :optimize-torque? nil))
    (print-sqp-result sqp-opt task-name-list)
    ;; 最適化計算が終了したら、robot-env-listを:dissoc-root-virtualして、仮想ルートジョイントを削除
    (send-all (send-all (send (send sqp-opt :config-task) :instant-config-task-list) :robot-env)
              :dissoc-root-virtual)

    sqp-opt))
(warn "(check-task *faucet-task* :kin-scale 1.0) ;; 単体タスク複数作業場所に対して固定された剛体脚配置を使って静力学の整合性をチェック~%")

;; roseus main.l '(main (list *faucet-task*) :set-viewpoint "faucet" :random-angle-vector-seed (list 51893784 1455289252) :kin-scale-list (list 0.6 0.6 0.01 0.01) :base-size-scale-list (list 0 (* 10.0 1e-4) (* 10.0 1e-4) (* 10.0 1e-4)) :fix-variant-joint-list (list nil nil nil nil) :fix-pad-diameter-list (list nil t nil nil))'
(defun main
    (tasks
     &key
     (pad-diameter 80)
     (random-angle-vector-seed nil)
     (kin-scale-list (list 1.0 1.0 0.01 0.01)) ;; デフォルトは1.0
     (base-size-scale-list (list 0 (* 10.0 1e-4) (* 10.0 1e-4) (* 10.0 1e-4))) ;; デフォルトは(* 10.0 1e-4)
     (fix-variant-joint-list (list nil nil nil t))
     (fix-pad-diameter-list (list nil t nil nil))
     (set-viewpoint)
     (debug-view t))
  "
 研究で使うメイン関数。吸着ベースの設計パラメータを時不変変数として扱っているのが特徴。

パラメータをグリッドサーチ際は、以下のようにコマンドライン引数からパラメータを変えられる。
シード値はなんでもいいわけではなくて、(make-random-state t)して得られた値を入れる。(make-random-state t)の返り値は時間依存。

前回最適化の結果を次の最適化の初期値にする際に、ホットスタートする理由
・sqp-opt内部では、:solve-qp-modeが:sqp-with-hotstartであり、qpoasesライブラリが内部で設定を継承することを想定している。
・前回最適化の結果を再利用しても、コールドスタート（プログラム再起動後など）だと、qpoases内部の設定が初期化されるため、うまくいかないことが多い。
"
  (let* ((robot (yamaguchi-arm :pad-diameter pad-diameter :transparency 0.4))
         (args (get-args robot tasks))
         (loop-num-min 30) (loop-num-max 50) ;; iteration回数は、loop-num-min以上、loop-num-max以下
         (task-name-list (flatten (mapcar #'(lambda(x) (make-list (send x :size) :initial-element (send x :name))) tasks)))
         sqp-opt)
    (unless (and (eq (length kin-scale-list) (length base-size-scale-list))
                 (eq (length kin-scale-list) (length fix-variant-joint-list)))
       (print "task settings are wrong.")
       (exit))

    ;; Visualize
    (when debug-view
      (visualize-tasks tasks)
      (when set-viewpoint
        (set-viewpoint set-viewpoint)))

    ;; Robot setting
    (if random-angle-vector-seed
        (progn
          (setq *random-state*
                (integer-vector (elt random-angle-vector-seed 0)
                                (elt random-angle-vector-seed 1)))
          (send robot :random-angle-vector))
      (send robot :move-rigid-leg-to-original))
    (if (elt fix-pad-diameter-list 0)
        (send robot :fix-pad-diameter)
      (send robot :unfix-pad-diameter))
    (send robot :newcoords (make-coords))

    ;; 1回目のOptimization
    ;; :inverse-kinematics-statics-trajectory-optmotiongen渡されている引数のうちいくつかは、robotオブジェクト内部のオブジェクトを参照している。
    ;; 以下のコメントにあるように、これら引数をselfに:putしてから(copy-object self)することで、robotオブジェクトではなくコピーされたオブジェクトの内部オブジェクトを参照できるようにしている
    ;; By putting cascaded coords and copying object, cascaded structure is holded in copied object and accessible by :get method.
    (setq sqp-opt (send robot
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
                        :collision-distance-limit -10 ;; 多少のめり込みを許容する？と、IKの成功率が格段に上がる[mm]
                        :invariant-joint-list (send robot :invariant-joint-list) ;; ベースの設計パラメータを固定
                        :temporary-invariant-joint-list (gethash :temporary-invariant-joint-list args) ;; ベース配置をタスクごとに固定
                        :adjacent-regular-scale-list (gethash :adjacent-regular-scale-list args)
                        :only-kinematics? nil
                        :kin-scale (car kin-scale-list)
                        :base-size-scale (car base-size-scale-list)
                        :initial-sqp-opt nil
                        :dissoc-root-virtual nil
                        :thre 3.0 ;; 手先の許容位置誤差
                        :debug-view (when debug-view (list :additional-draw-objects (send *irtviewer* :objects)))
                        :revert-if-fail nil
                        :root-virtual-mode :6dof
                        :min-loop loop-num-min
                        :stop loop-num-max
                        :optimize-torque? nil))
    (print-sqp-result sqp-opt task-name-list :debug-view debug-view)

    ;; 2回目以降のOptimization
    (dotimes (i (- (length kin-scale-list) 1))
      ;; 段階的最適化をするため、前回の最適化結果であるsqp-optのパラメータを変えてから、次の最適化の初期値にする
      ;; TODO: eom-scaleも変えられるようにする（そもそも必要ないかもしれない）
      (let* ((config-task-list (send (send sqp-opt :config-task) :instant-config-task-list)))
        ;; (send-all config-task-list :only-kinematics? nil)
        (send-all config-task-list :kin-scale (elt kin-scale-list (+ i 1)))
        (send-all config-task-list :base-size-scale (elt base-size-scale-list (+ i 1)))
        (let ((robot-list (send-all (send-all config-task-list :robot-env) :robot)))
          (if (elt fix-pad-diameter-list (+ i 1))
              (send-all robot-list :fix-pad-diameter)
            (send-all robot-list :unfix-pad-diameter))
          (send-all config-task-list :phi-max-vector :update? t)
          (send-all config-task-list :phi-min-vector :update? t))
        (if (elt fix-variant-joint-list (+ i 1))
            (let* ((robot-env-list (send-all config-task-list :robot-env))
                   (arm-joint-list (send-all (send-all robot-env-list :robot) :variant-joint-list))
                   (root-virtual-joint-list (send-all robot-env-list :root-virtual-joint-list))
                   (orig-max-angle-list (mapcar #'(lambda (j) (send-all j :max-angle)) arm-joint-list))
                   (orig-min-angle-list (mapcar #'(lambda (j) (send-all j :min-angle)) arm-joint-list)))
              ;; fix variant joint angle
              (mapcar #'(lambda (jl) (mapcar #'(lambda (j) (send j :max-angle (send j :joint-angle))) jl)) arm-joint-list)
              (mapcar #'(lambda (jl) (mapcar #'(lambda (j) (send j :min-angle (send j :joint-angle))) jl)) arm-joint-list)
              (mapcar #'(lambda (jl) (mapcar #'(lambda (j) (send j :max-angle (send j :joint-angle))) jl)) root-virtual-joint-list)
              (mapcar #'(lambda (jl) (mapcar #'(lambda (j) (send j :min-angle (send j :joint-angle))) jl)) root-virtual-joint-list)
              (send-all config-task-list :theta-max-vector :update? t)
              (send-all config-task-list :theta-min-vector :update? t)
              (send sqp-opt :optimize :loop-num loop-num-max :loop-num-min loop-num-min)
              ;; unfix variant joint angle
              (mapcar #'(lambda (jl max-angle-list) (mapcar #'(lambda (j m) (send j :max-angle m)) jl max-angle-list))
                      arm-joint-list orig-max-angle-list)
              (mapcar #'(lambda (jl min-angle-list) (mapcar #'(lambda (j m) (send j :min-angle m)) jl min-angle-list))
                      arm-joint-list orig-min-angle-list)
              (mapcar #'(lambda (jl) (mapcar #'(lambda (j) (send j :max-angle 1e10)) jl)) root-virtual-joint-list)
              (mapcar #'(lambda (jl) (mapcar #'(lambda (j) (send j :min-angle -1e10)) jl)) root-virtual-joint-list)
              (send-all config-task-list :theta-max-vector :update? t)
              (send-all config-task-list :theta-min-vector :update? t)
              )
          (send sqp-opt :optimize :loop-num loop-num-max :loop-num-min loop-num-min)))
      (print-sqp-result sqp-opt task-name-list :debug-view debug-view))

    ;; 最適化計算が終了したら、robot-env-listを:dissoc-root-virtualして、仮想ルートジョイントを削除
    (send-all (send-all (send (send sqp-opt :config-task) :instant-config-task-list) :robot-env)
              :dissoc-root-virtual)

    sqp-opt))
(warn "(main (list *faucet-task*)) ;; trajectoryタスクに対して剛体脚の設計を出力~%")
