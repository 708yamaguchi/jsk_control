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
  (let* ((robot (yamaguchi-arm :pad-diameter 60))
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
    (print-sqp-result sqp-opt args :dissoc-root-virtual nil)

    sqp-opt))
(warn "(check-single *faucet-task*) ;; 単体作業場所に対して固定された剛体脚配置を使って静力学の整合性をチェック~%")

(defun check-task
    (task &key (kin-scale 1.0))
  (let* ((robot (yamaguchi-arm :pad-diameter 60))
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
    (print-sqp-result sqp-opt args)
    ;; 最適化計算が終了したら、robot-env-listを:dissoc-root-virtualして、仮想ルートジョイントを削除
    (send-all (send-all (send (send sqp-opt :config-task) :instant-config-task-list) :robot-env)
              :dissoc-root-virtual)

    sqp-opt))
(warn "(check-task *faucet-task* :kin-scale 1.0) ;; 単体タスク複数作業場所に対して固定された剛体脚配置を使って静力学の整合性をチェック~%")

;; yamaguchi_6axis_arm_nejineji_short
;; roseus main.l '(main (list *faucet-task*) :set-viewpoint "faucet" :random-angle-vector-seed (list 860217820 1488608567) :kin-scale-list (list 0.4 0.4 0.4 0.4 0.4 0.4) :base-size-scale-list (list 0 (* 4.0 1e-4) (* 6.0 1e-4) (* 8.0 1e-4) (* 10.0 1e-4) (* 10.0 1e-4)) :fix-variant-joint-list (list nil nil nil nil nil nil))'
;; -> (send robot :angle-vector #f(0 0 0 0 -90 0 0.0 0.0 10.3848 93.1552 10.3872 93.1551 10.3927 93.155 10.3903 93.1551 0.0 0.0 0.0))
;; -> Base convex hull (#f(-30.0 3.673940e-15 0.0) #f(-21.2132 -21.2132 0.0) #f(-5.880162e-14 -30.0 0.0) #f(21.2132 -21.2132 0.0) #f(30.0 0.0 0.0) #f(10.3927 93.155 0.0) #f(10.3903 93.1551 0.0) #f(10.3848 93.1552 0.0) #f(-21.2132 21.2132 0.0))
;; -> 4086.89 [mm^2]
;; roseus main.l '(main (list *faucet-wall-task*) :set-viewpoint "faucet" :random-angle-vector-seed (list 1777581173 1619544386) :kin-scale-list (list 0.4 0.4 0.4 0.4 0.4 0.4) :base-size-scale-list (list 0 (* 4.0 1e-4) (* 6.0 1e-4) (* 8.0 1e-4) (* 10.0 1e-4) (* 10.0 1e-4)) :fix-variant-joint-list (list nil nil nil nil nil nil))'
;; -> (send robot :angle-vector #f(0 0 0 0 -90 0 0.0 0.0 -21.2419 -25.0552 -35.6628 -27.3711 -73.6238 -36.662 -72.1304 -36.9323 0.0 0.0 0.0))
;; -> Base convex hull (#f(-73.6238 -36.662 0.0) #f(-72.1304 -36.9323 0.0) #f(-5.880162e-14 -30.0 0.0) #f(21.2132 -21.2132 0.0) #f(30.0 0.0 0.0) #f(21.2132 21.2132 0.0) #f(1.836970e-15 30.0 0.0) #f(-21.2132 21.2132 0.0))
;; -> 3880.03 [mm^2]

;; roseus main.l '(main (list *ih-task*) :set-viewpoint "IH" :random-angle-vector-seed (list 742790772 227240069) :kin-scale-list (list 0.4 0.4 0.4 0.4 0.4 0.4) :base-size-scale-list (list 0 (* 4.0 1e-4) (* 6.0 1e-4) (* 8.0 1e-4) (* 10.0 1e-4) (* 10.0 1e-4)) :fix-variant-joint-list (list nil nil nil nil nil nil))'
;; -> (send robot :angle-vector #f(0 0 0 0 -90 0 0.0 0.0 -43.6038 11.4783 -44.2986 11.459 -44.364 11.5106 -44.3771 11.483 0.0 0.0 0.0 0.0 0.0 0.0))
;; -> Base convex hull (#f(-44.3771 11.483 0.0) #f(-21.2132 -21.2132 0.0) #f(-5.880162e-14 -30.0 0.0) #f(21.2132 -21.2132 0.0) #f(30.0 0.0 0.0) #f(21.2132 21.2132 0.0) #f(1.836970e-15 30.0 0.0) #f(-21.2132 21.2132 0.0) #f(-44.364 11.5106 0.0))
;; -> 2850.82 [mm^2]

;; yamaguchi_6axis_arm_nejineji
;; roseus main.l '(main (list *wipe-ih-wall-task*) :set-viewpoint "IH" :random-angle-vector-seed (list 860217820 1488608567) :kin-scale-list (list 0.4 0.4 0.4 0.4 0.4 0.4) :base-size-scale-list (list 0 (* 4.0 1e-4) (* 6.0 1e-4) (* 8.0 1e-4) (* 10.0 1e-4) (* 10.0 1e-4)) :fix-variant-joint-list (list nil nil nil nil nil nil))'
;; -> (send robot :angle-vector #f(0 0 0 0 -90 0 0.0 0.0 -13.7709 -91.0396 -10.6108 149.8 19.6753 165.633 -19.5571 -143.868 0.0 0.0 0.0 0.0 0.0 0.0))
;; -> Base convex hull (#f(-30.0 3.673940e-15 0.0) #f(-19.5571 -143.868 0.0) #f(30.0 0.0 0.0) #f(19.6753 165.633 0.0) #f(-10.6108 149.8 0.0))
;; -> 11400.0 [mm^2]

;; yamaguchi_4axis_arm_nejineji_short
;; roseus main.l '(main (list *microwave-task*) :set-viewpoint "microwave" :random-angle-vector-seed (list 1654553012 786543381) :kin-scale-list (list 0.4 0.4 0.4 0.4 0.4 0.4) :base-size-scale-list (list 0 (* 4.0 1e-4) (* 6.0 1e-4) (* 8.0 1e-4) (* 10.0 1e-4) (* 10.0 1e-4)) :fix-variant-joint-list (list nil nil nil nil nil nil))'
;; -> (send robot :angle-vector #f(0 0 0 0 0.0 0.0 -33.5556 -36.3041 -26.546 -28.5806 -42.6717 -46.4694 -35.7073 -38.6917 0.0 0.0 0.0))
;; Base convex hull (#f(-42.6717 -46.4694 0.0) #f(-5.880162e-14 -30.0 0.0) #f(21.2132 -21.2132 0.0) #f(30.0 0.0 0.0) #f(21.2132 21.2132 0.0) #f(1.836970e-15 30.0 0.0) #f(-21.2132 21.2132 0.0) #f(-30.0 3.673940e-15 0.0))
;; -> 3246.3 [mm^2]

;; Draw base
;; (draw-base-convex-hull robot :v *irtviewer*)
(defun main
    (tasks
     &key
     (pad-diameter 60)
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
    ;; (unix:sleep 3)

    ;; Robot setting
    (send robot :fix-gripper)
    (if random-angle-vector-seed
        (progn
          (setq *random-state*
                (integer-vector (elt random-angle-vector-seed 0)
                                (elt random-angle-vector-seed 1)))
          (send robot :random-angle-vector))
      (send robot :move-rigid-leg-to-original))
    ;; (if (elt fix-pad-diameter-list 0)
    ;;     (send robot :fix-pad-diameter)
    ;;   (send robot :unfix-pad-diameter))
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
                        :collision-distance-limit 5 ;; 多少のめり込みを許容する？と、IKの成功率が格段に上がる[mm]
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
    (print-sqp-result sqp-opt args :debug-view nil)

    ;; 2回目以降のOptimization
    (dotimes (i (- (length kin-scale-list) 1))
      ;; (when (eq i (- (length kin-scale-list) 2))
      ;;   (break))
      ;; 段階的最適化をするため、前回の最適化結果であるsqp-optのパラメータを変えてから、次の最適化の初期値にする
      ;; TODO: eom-scaleも変えられるようにする（そもそも必要ないかもしれない）
      (let* ((config-task-list (send (send sqp-opt :config-task) :instant-config-task-list)))
        ;; (send-all config-task-list :only-kinematics? nil)
        (send-all config-task-list :kin-scale (elt kin-scale-list (+ i 1)))
        (send-all config-task-list :base-size-scale (elt base-size-scale-list (+ i 1)))
        (let ((robot-list (send-all (send-all config-task-list :robot-env) :robot)))
          ;; (if (elt fix-pad-diameter-list (+ i 1))
          ;;     (send-all robot-list :fix-pad-diameter)
          ;;   (send-all robot-list :unfix-pad-diameter))
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
      (print-sqp-result sqp-opt args :debug-view nil))

    ;; 最適化計算が終了したら、robot-env-listを:dissoc-root-virtualして、仮想ルートジョイントを削除
    (send-all (send-all (send (send sqp-opt :config-task) :instant-config-task-list) :robot-env)
              :dissoc-root-virtual)

    sqp-opt))
(warn "(main (list *faucet-task*)) ;; trajectoryタスクに対して剛体脚の設計を出力~%")
