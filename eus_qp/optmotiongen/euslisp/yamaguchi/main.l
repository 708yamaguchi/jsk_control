(load "../inverse-kinematics-statics-wrapper.l")
(load "../discrete-kinematics-configuration-task.l")
(load "../sqp-msc-optimization.l")

(load "room610-model-list.l") ;; Set *room610-model-list*
(load "task.l") ;; Set *tasks*
(load "util.l")
(load "yamaguchi-arm.l") ;; Define (yamaguchi-arm) function
(load "contact-constrant-models.l")


;; (defun check-single
;;     (task)
;;   "
;; ベースの形状を固定にして:inverse-kinematics-statics-optmotiongenを解いて、静力学的に解が出るかをチェックする
;; 現実と同じ物理パラメータになっているかをチェックできるようにするのが目的。

;; 引数のtaskに定義されているcoordsやwrenchのうち、最初の要素を利用してチェック
;; "
;;   (let* ((robot (yamaguchi-arm :pad-diameter 60))
;;          (args (get-args robot (list task)))
;;          sqp-opt)

;;     ;; Visualize
;;     (visualize-tasks (list task))

;;     ;; Robot settings
;;     (send robot :move-rigid-leg-to-original)
;;     (send robot :fix-leg) ;; 剛体脚の位置を固定する
;;     (send robot :newcoords (make-coords))

;;     ;; Optimization
;;     (setq sqp-opt (send robot
;;                         :inverse-kinematics-statics-optmotiongen
;;                         (car (gethash :target-coords-list args))
;;                         :move-target (car (gethash :move-target-list args))
;;                         :contact-target-coords (car (gethash :contact-target-coords-list args))
;;                         :contact-move-target (car (gethash :contact-move-target-list args))
;;                         :contact-constraint (car (gethash :contact-constraint-list args))
;;                         :external-wrench-list (car (gethash :external-wrench-list args))
;;                         :external-coords-list (car (gethash :external-coords-list args))
;;                         :translation-axis (car (gethash :translation-axis-list args))
;;                         :rotation-axis (car (gethash :rotation-axis-list args))
;;                         :collision-avoidance-link-pair (gethash :collision-avoidance-link-pair args)
;;                         :collision-distance-limit -10 ;; 多少のめり込みを許容する？と、IKの成功率が格段に上がる[mm]
;;                         :debug-view (list :additional-draw-objects (send *irtviewer* :objects))
;;                         :revert-if-fail nil
;;                         :root-virtual-mode :6dof
;;                         :stop 100
;;                         :optimize-torque? nil))

;;     ;; Print result
;;     (print-sqp-result sqp-opt args :dissoc-root-virtual nil)

;;     sqp-opt))
;; (warn "(check-single *faucet-task*) ;; 単体作業場所に対して固定された剛体脚配置を使って静力学の整合性をチェック~%")

;; (defun check-task
;;     (task &key (kin-scale 1.0))
;;   (let* ((robot (yamaguchi-arm :pad-diameter 60))
;;          (args (get-args robot (list task)))
;;          (loop-num-min 30) (loop-num-max 100) ;; iteration回数は、loop-num-min以上、loop-num-max以下
;;          (task-name-list (make-list (send task :size) :initial-element (send task :name)))
;;          sqp-opt)

;;     ;; Visualize
;;     (visualize-tasks (list task))

;;     ;; Robot setting
;;     (send robot :move-rigid-leg-to-original)
;;     (send robot :fix-leg)
;;     (send robot :newcoords (make-coords))

;;     (setq sqp-opt (send robot
;;                         :inverse-kinematics-statics-trajectory-optmotiongen
;;                         (gethash :target-coords-list args)
;;                         :move-target-list (gethash :move-target-list args)
;;                         :contact-target-coords-list (gethash :contact-target-coords-list args)
;;                         :contact-move-target-list (gethash :contact-move-target-list args)
;;                         :contact-constraint-list (gethash :contact-constraint-list args)
;;                         :external-wrench-list (gethash :external-wrench-list args)
;;                         :external-coords-list (gethash :external-coords-list args)
;;                         :translation-axis-list (gethash :translation-axis-list args)
;;                         :rotation-axis-list (gethash :rotation-axis-list args)
;;                         :collision-avoidance-link-pair (gethash :collision-avoidance-link-pair args)
;;                         :collision-distance-limit 0 ;; 多少のめり込みを許容する？と、IKの成功率が格段に上がる[mm]
;;                         ;; :invariant-joint-list (send robot :invariant-joint-list) ;; ベースの設計パラメータを固定
;;                         :temporary-invariant-joint-list (gethash :temporary-invariant-joint-list args) ;; ベース配置をタスクごとに固定
;;                         :adjacent-regular-scale-list (gethash :adjacent-regular-scale-list args)
;;                         :only-kinematics? nil
;;                         :kin-scale kin-scale
;;                         :base-size-scale 0 ;; invariant-joint-listがないので、そもそもbase taskがないが、分かりやすさのため0にする。
;;                         :dissoc-root-virtual nil
;;                         :thre 5.0 ;; 手先の許容位置誤差
;;                         :debug-view (list :additional-draw-objects (send *irtviewer* :objects))
;;                         :revert-if-fail nil
;;                         :root-virtual-mode :6dof
;;                         :min-loop loop-num-min
;;                         :stop loop-num-max
;;                         :optimize-torque? nil))
;;     (print-sqp-result sqp-opt args)
;;     ;; 最適化計算が終了したら、robot-env-listを:dissoc-root-virtualして、仮想ルートジョイントを削除
;;     (send-all (send-all (send (send sqp-opt :config-task) :instant-config-task-list) :robot-env)
;;               :dissoc-root-virtual)

;;     sqp-opt))
;; (warn "(check-task *faucet-task* :kin-scale 1.0) ;; 単体タスク複数作業場所に対して固定された剛体脚配置を使って静力学の整合性をチェック~%")

;; yamaguchi_6axis_arm_nejineji_short
;; roseus main.l '(main (list *faucet-task*) :set-viewpoint "faucet" :random-angle-vector-seed (list 860217820 1488608567) :kin-scale-list (list 0.4 0.4 0.4 0.4 0.4 0.4) :base-size-scale-list (list 0 (* 4.0 1e-4) (* 6.0 1e-4) (* 8.0 1e-4) (* 10.0 1e-4) (* 10.0 1e-4)) :fix-variant-joint-list (list nil nil nil nil nil nil))'
;; -> (send robot :angle-vector #f(0 0 0 0 -90 0 0.0 0.0 10.3848 93.1552 10.3872 93.1551 10.3927 93.155 10.3903 93.1551 0.0 0.0 0.0))
;; -> Base convex hull (#f(-30.0 3.673940e-15 0.0) #f(-21.2132 -21.2132 0.0) #f(-5.880162e-14 -30.0 0.0) #f(21.2132 -21.2132 0.0) #f(30.0 0.0 0.0) #f(10.3927 93.155 0.0) #f(10.3903 93.1551 0.0) #f(10.3848 93.1552 0.0) #f(-21.2132 21.2132 0.0))
;; -> 4086.89 [mm^2]
;; |task-all|=0.265124
;; 蛇口（平面）タスク：
;; 姿勢1：
;;   e_pos: 0.002024
;;   e_wrench(trans): 8.102424e-06 [N]
;;   e_wrench(rot): 6.672572e-05 [Nm]
;;   e_base: 0.187465
;;   e: 0.187476
;; 姿勢2：
;;   e_pos: 5.246524e-05
;;   e_wrench(trans): 3.139078e-08 [N]
;;   e_wrench(rot): 3.231539e-07 [Nm]
;;   e_base: 0.187465
;;   e: 0.187465
;; Intel(R) Core(TM) i9-13900KFでの計算時間：13.86 [s] (:debug-view nil)

;; roseus main.l '(main (list *faucet-wall-task*) :set-viewpoint "faucet" :random-angle-vector-seed (list 1777581173 1619544386) :kin-scale-list (list 0.4 0.4 0.4 0.4 0.4 0.4) :base-size-scale-list (list 0 (* 4.0 1e-4) (* 6.0 1e-4) (* 8.0 1e-4) (* 10.0 1e-4) (* 10.0 1e-4)) :fix-variant-joint-list (list nil nil nil nil nil nil))'
;; -> (send robot :angle-vector #f(0 0 0 0 -90 0 0.0 0.0 -21.2419 -25.0552 -35.6628 -27.3711 -73.6238 -36.662 -72.1304 -36.9323 0.0 0.0 0.0))
;; -> Base convex hull (#f(-73.6238 -36.662 0.0) #f(-72.1304 -36.9323 0.0) #f(-5.880162e-14 -30.0 0.0) #f(21.2132 -21.2132 0.0) #f(30.0 0.0 0.0) #f(21.2132 21.2132 0.0) #f(1.836970e-15 30.0 0.0) #f(-21.2132 21.2132 0.0))
;; -> 3880.03 [mm^2]
;; |task-all|=0.181311
;; 蛇口（壁面）タスク：
;; 姿勢1：
;;   e_pos: 0.002826
;;   e_wrench(trans): 2.864730e-06 [N]
;;   e_wrench(rot): 1.650252e-05 [Nm]
;;   e_base: 0.128185
;;   e: 0.128216
;; 姿勢2：
;;   e_pos: 0.001692
;;   e_wrench(trans): 1.484045e-06 [N]
;;   e_wrench(rot): 4.529604e-05 [Nm]
;;   e_base: 0.128185
;;   e: 0.128196
;; Intel(R) Core(TM) i9-13900KFでの計算時間：13.86 [s] (:debug-view nil)

;; roseus main.l '(main (list *ih-task*) :set-viewpoint "IH" :random-angle-vector-seed (list 742790772 227240069) :kin-scale-list (list 0.4 0.4 0.4 0.4 0.4 0.4) :base-size-scale-list (list 0 (* 4.0 1e-4) (* 6.0 1e-4) (* 8.0 1e-4) (* 10.0 1e-4) (* 10.0 1e-4)) :fix-variant-joint-list (list nil nil nil nil nil nil))'
;; -> (send robot :angle-vector #f(0 0 0 0 -90 0 0.0 0.0 -43.6038 11.4783 -44.2986 11.459 -44.364 11.5106 -44.3771 11.483 0.0 0.0 0.0 0.0 0.0 0.0))
;; -> Base convex hull (#f(-44.3771 11.483 0.0) #f(-21.2132 -21.2132 0.0) #f(-5.880162e-14 -30.0 0.0) #f(21.2132 -21.2132 0.0) #f(30.0 0.0 0.0) #f(21.2132 21.2132 0.0) #f(1.836970e-15 30.0 0.0) #f(-21.2132 21.2132 0.0) #f(-44.364 11.5106 0.0))
;; -> 2850.82 [mm^2]

;; yamaguchi_6axis_arm_nejineji
;; roseus main.l '(main (list *wipe-ih-wall-task*) :set-viewpoint "IH" :random-angle-vector-seed (list 860217820 1488608567) :kin-scale-list (list 0.4 0.4 0.4 0.4 0.4 0.4) :base-size-scale-list (list 0 (* 4.0 1e-4) (* 6.0 1e-4) (* 8.0 1e-4) (* 10.0 1e-4) (* 10.0 1e-4)) :fix-variant-joint-list (list nil nil nil nil nil nil))'
;; -> (send robot :angle-vector #f(0 0 0 0 -90 0 0.0 0.0 -13.7709 -91.0396 -10.6108 149.8 19.6753 165.633 -19.5571 -143.868 0.0 0.0 0.0 0.0 0.0 0.0))
;; -> Base convex hull (#f(-30.0 3.673940e-15 0.0) #f(-19.5571 -143.868 0.0) #f(30.0 0.0 0.0) #f(19.6753 165.633 0.0) #f(-10.6108 149.8 0.0))
;; -> 11400.0 [mm^2]
;; |task-all|=0.565461
;; 姿勢1：
;;   e_pos: 0.002522
;;   e_wrench(trans): 1.580613e-05 [N]
;;   e_wrench(rot): 7.658116e-05 [Nm]
;;   e_base: 0.282723
;;   e: 0.282734
;; 姿勢2：
;;   e_pos: 0.002447
;;   e_wrench(trans): 1.169996e-05 [N]
;;   e_wrench(rot): 9.817100e-05 [Nm]
;;   e_base: 0.282723
;;   e: 0.282734
;; 姿勢3：
;;   e_pos: 0.002175
;;   e_wrench(trans): 4.612754e-06 [N]
;;   e_wrench(rot): 5.308761e-05 [Nm]
;;   e_base: 0.282723
;;   e: 0.282731
;; 姿勢4：
;;   e_pos: 0.000337
;;   e_wrench(trans): 1.394419e-05 [N]
;;   e_wrench(rot): 8.987842e-06 [Nm]
;;   e_base: 0.282723
;;   e: 0.282723
;; Intel(R) Core(TM) i9-13900KFでの計算時間：40.98 [s] (:debug-view nil)

;; yamaguchi_4axis_arm_nejineji_short
;; roseus main.l '(main (list *microwave-task*) :set-viewpoint "microwave" :random-angle-vector-seed (list 1654553012 786543381) :kin-scale-list (list 0.4 0.4 0.4 0.4 0.4 0.4) :base-size-scale-list (list 0 (* 4.0 1e-4) (* 6.0 1e-4) (* 8.0 1e-4) (* 10.0 1e-4) (* 10.0 1e-4)) :fix-variant-joint-list (list nil nil nil nil nil nil))'
;; -> (send robot :angle-vector #f(0 0 0 0 0.0 0.0 -33.5556 -36.3041 -26.546 -28.5806 -42.6717 -46.4694 -35.7073 -38.6917 0.0 0.0 0.0))
;; Base convex hull (#f(-42.6717 -46.4694 0.0) #f(-5.880162e-14 -30.0 0.0) #f(21.2132 -21.2132 0.0) #f(30.0 0.0 0.0) #f(21.2132 21.2132 0.0) #f(1.836970e-15 30.0 0.0) #f(-21.2132 21.2132 0.0) #f(-30.0 3.673940e-15 0.0))
;; -> 3246.3 [mm^2]
;; 電子レンジタスク：
;; |task-all|=0.146411
;; 姿勢1：
;;   e_pos: 2.981672e-05
;;   e_wrench(trans): 6.796077e-07 [N]
;;   e_wrench(rot): 2.786066e-07 [Nm]
;;   e_base: 0.103527
;;   e: 0.103527
;; 姿勢2：
;;   e_pos: 0.00079
;;   e_wrench(trans): 2.889290e-06 [N]
;;   e_wrench(rot): 4.614444e-05 [Nm]
;;   e_base: 0.103527
;;   e: 0.10353
;; Intel(R) Core(TM) i9-13900KFでの計算時間：10.91 [s] (:debug-view nil)

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
     (set-viewpoint) (debug-view t) (start-time))
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

    (setq start-time (car (unix:ptimes)))
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
    (format t "Optimization 1: elapsed time ~A [s]~%" (* 0.01 (- (car (unix:ptimes)) start-time)))
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
      (format t "Optimization ~A: elapsed time ~A [s]~%" (+ i 2) (* 0.01 (- (car (unix:ptimes)) start-time)))
      (print-sqp-result sqp-opt args :debug-view nil))

    ;; 最適化計算が終了したら、robot-env-listを:dissoc-root-virtualして、仮想ルートジョイントを削除
    (send-all (send-all (send (send sqp-opt :config-task) :instant-config-task-list) :robot-env)
              :dissoc-root-virtual)

    sqp-opt))
(warn "(main (list *faucet-task*)) ;; trajectoryタスクに対して剛体脚の設計を出力~%")

(defun check-success
    (tasks
     &key
     (pad-diameter 60)
     (random-angle-vector-seed nil)
     (kin-scale-list (list 1.0 1.0 0.01 0.01)) ;; デフォルトは1.0
     (base-size-scale-list (list 0 (* 10.0 1e-4) (* 10.0 1e-4) (* 10.0 1e-4))) ;; デフォルトは(* 10.0 1e-4)
     (fix-variant-joint-list (list nil nil nil t))
     (fix-pad-diameter-list (list nil t nil nil))
     (set-viewpoint) (debug-view nil) (start-time))
  "
(main)関数とほとんど同じ。一回でも最適化に成功したら、tを返す。最後まで失敗したら、nilを返す。
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

    (setq start-time (car (unix:ptimes)))
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
    (when (eq (send sqp-opt :status) :success)
      (format t "Optimization succeeded.~%")
      (return-from check-success t))
    (format t "Optimization 1: elapsed time ~A [s]~%" (* 0.01 (- (car (unix:ptimes)) start-time)))
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
      (when (eq (send sqp-opt :status) :success)
        (format t "Optimization succeeded.~%")
        (return-from check-success t))
      (format t "Optimization ~A: elapsed time ~A [s]~%" (+ i 2) (* 0.01 (- (car (unix:ptimes)) start-time)))
      (print-sqp-result sqp-opt args :debug-view nil))

    ;; 最適化計算が終了したら、robot-env-listを:dissoc-root-virtualして、仮想ルートジョイントを削除
    (send-all (send-all (send (send sqp-opt :config-task) :instant-config-task-list) :robot-env)
              :dissoc-root-virtual)

    sqp-opt

    (format t "Optimization failed.~%")
    nil))


(defun check-success-cl (task-name seed-id)
  "
(make-random-state t)して、初期seedを得る
タスクごとに、yamaguchi-armの継承元クラスを変える
"
  ;; roseus main.l '(check-success-cl "microwave" 0)'
  ;; echo $?
  ;; 0 or 255

  (let* ((seeds (list
                 ;; 0
                 (list 1048567774 1899617731)
                 (list 751032584 644330867)
                 (list 1511076385 1521795317)
                 (list 141833841 268266119)
                 (list 1975916939 75263090)
                 (list 594851123 2031278026)
                 (list 1365248517 1845830322)
                 (list 2147084298 1667729295)
                 (list 763156496 406133717)
                 (list 1528592442 212821502)
                 ;; 10
                 (list 1228756572 31108727)
                 (list 918296004 1985461814)
                 (list 1693999758 1803191333)
                 (list 320279119 1622264707)
                 (list 1083963718 356081527)
                 (list 1858328932 1246290212)
                 (list 1548168129 1058476440)
                 (list 175517671 875101433)
                 (list 942733910 686200558)
                 (list 1714034513 500409287)
                 ;; 20
                 (list 318885980 1371998357)
                 (list 1098887823 119122986)
                 (list 1865771042 999776368)
                 (list 496650336 829638325)
                 (list 180122626 628395377)
                 (list 2033533027 452931541)
                 (list 1736354129 271593723)
                 (list 1427424226 77413545)
                 (list 1117663640 2033836657)
                 (list 817845805 1852370238)
                 ;; 30
                 (list 519159532 595319270)
                 (list 1291537336 413296864)
                 (list 2057018900 223825775)
                 (list 666724700 1097766349)
                 (list 372173637 921301581)
                 (list 67085833 731030788)
                 (list 1908695006 1619241635)
                 (list 523488699 1423614785)
                 (list 234224755 180892392)
                 (list 2068605912 1056656246)
                 ;; 40
                 (list 1765960291 872897559)
                 (list 387404379 683917509)
                 (list 1158824786 1572545862)
                 (list 1925109981 1382633271)
                 (list 557522928 1202675427)
                 (list 1316642862 1011835135)
                 (list 2083186563 1890477935)
                 (list 714507211 1716705143)
                 (list 406572269 1524993909)
                 (list 103778336 1338832901)
                 ;; 50
                 (list 870976231 1148988207)
                 (list 1644274024 963943928)
                 (list 255969103 1841349298)
                 (list 1042862866 1674490683)
                 (list 737395355 411469546)
                 (list 439559373 233419978)
                 (list 1205272866 1116438605)
                 (list 1965110190 1994418232)
                 (list 587637771 1806166648)
                 (list 283618204 544584058)
                 ;; 60
                 (list 2132422449 1438270942)
                 (list 1818651852 1243959108)
                 (list 1519271638 2134993855)
                 (list 149125617 1953866498)
                 (list 1988769679 691062765)
                 (list 603033274 497543286)
                 (list 304528962 1391368142)
                 (list 2142640359 1193334219)
                 (list 1844360935 1015607887)
                 (list 461264980 1899304690)
                 ;; 70
                 (list 148654344 630745732)
                 (list 933648868 457784743)
                 (list 627840431 1341998933)
                 (list 318141808 80348330)
                 (list 6022203 956441307)
                 (list 1859655401 779695420)
                 (list 486414569 1673213740)
                 (list 1252645466 409836221)
                 (list 2025797277 227399004)
                 (list 1726427148 44704715)
                 ;; 80
                 (list 332667030 1986544423)
                 (list 28520676 728698195)
                 (list 808090168 547761391)
                 (list 1573903881 1436624094)
                 (list 204264531 182874096)
                 (list 2044924065 1065927045)
                 (list 662801185 877739856)
                 (list 1432329186 688664860)
                 (list 1132233129 1579282922)
                 (list 820118370 1384269502)
                 ;; 90
                 (list 1593516345 1202743317)
                 (list 1294299867 1021737124)
                 (list 2061409153 1906901178)
                 (list 1752276993 1714711559)
                 (list 376882932 1529574430)
                 (list 1144576712 1340539042)
                 (list 839543982 79602560)
                 (list 541255875 2045253719)
                 (list 1295753368 1844680491)
                 (list 1003786670 598252455)
                 ;; 100
                 ))
         (seed (elt seeds seed-id))
         (success-num 0)
         (tasks-done 0)
         result)
    (setq result
          (cond
           ;; yamaguchi_4axis_arm_nejineji_short
           ((equal task-name "microwave")
            (check-success (list *microwave-task*) :set-viewpoint "microwave" :random-angle-vector-seed (list (car seed) (cadr seed)) :kin-scale-list (list 0.4 0.4 0.4 0.4 0.4 0.4) :base-size-scale-list (list 0 (* 4.0 1e-4) (* 6.0 1e-4) (* 8.0 1e-4) (* 10.0 1e-4) (* 10.0 1e-4)) :fix-variant-joint-list (list nil nil nil nil nil nil)))

           ;; yamaguchi_6axis_arm_nejineji
           ((equal task-name "wipe-ih-wall")
            (check-success (list *wipe-ih-wall-task*) :set-viewpoint "IH" :random-angle-vector-seed (list (car seed) (cadr seed)) :kin-scale-list (list 0.4 0.4 0.4 0.4 0.4 0.4) :base-size-scale-list (list 0 (* 4.0 1e-4) (* 6.0 1e-4) (* 8.0 1e-4) (* 10.0 1e-4) (* 10.0 1e-4)) :fix-variant-joint-list (list nil nil nil nil nil nil)))

           ;; yamaguchi_6axis_arm_nejineji_short
           ((equal task-name "faucet")
            (check-success (list *faucet-task*) :set-viewpoint "faucet" :random-angle-vector-seed (list (car seed) (cadr seed)) :kin-scale-list (list 0.4 0.4 0.4 0.4 0.4 0.4) :base-size-scale-list (list 0 (* 4.0 1e-4) (* 6.0 1e-4) (* 8.0 1e-4) (* 10.0 1e-4) (* 10.0 1e-4)) :fix-variant-joint-list (list nil nil nil nil nil nil)))
           ((equal task-name "faucet-wall")
            (check-success (list *faucet-wall-task*) :set-viewpoint "faucet" :random-angle-vector-seed (list (car seed) (cadr seed)) :kin-scale-list (list 0.4 0.4 0.4 0.4 0.4 0.4) :base-size-scale-list (list 0 (* 4.0 1e-4) (* 6.0 1e-4) (* 8.0 1e-4) (* 10.0 1e-4) (* 10.0 1e-4)) :fix-variant-joint-list (list nil nil nil nil nil nil)))
           ))
    (if result
        (exit 0)
      (exit 255))
    ))
