(load "attachable-plane.l")

(defclass task
  :super propertied-object
  :slots (_name _attachable-plane-list _coords-list _wrench-list _size)
  )
(defmethod task
  (:init
   (&key (name) (attachable-plane-obj) (coords-list) (wrench-list))
   (setq _name name)
   (setq _coords-list coords-list)
   (send-all _coords-list :name _name)
   (setq _wrench-list wrench-list)
   (if (eq (length coords-list) (length wrench-list))
       (setq _size (length coords-list))
     (progn
       (print "Task definition is wrong.")
       (exit)))
   ;; 全く同じ位置の平面は与えられなかったので、同じ場所で複数タスクを行う場合は微小量だけ動かした平面を利用
   (dotimes (i _size)
     (let ((p (copy-object attachable-plane-obj)))
       (send p :offset (+ (send p :offset) (* i 0.001)))
       (setq _attachable-plane-list (append _attachable-plane-list (list p)))))
   )
  (:size
   ()
   _size
   )
  (:name
   ()
   _name)
  (:attachable-plane-list
   ()
   _attachable-plane-list)
  (:coords-list
   ()
   _coords-list)
  (:wrench-list
   ()
   _wrench-list)
  )

;; TODO: タスクに合っている力の制約を入れる。タスクの場所も増やす
;; wrenchを少し変えるだけでも最適化の成否が変わるのが難しい
(setq *faucet-task*
      (instance task :init
                :name "faucet"
                :attachable-plane-obj
                (instance attachable-plane :init
                          :offset 900
                          :min-angle-list (list 7730 -270)
                          :max-angle-list (list 8320 590)
                          :normal :z+)
                :coords-list
                (list
                 (make-cascoords :pos (float-vector 8100 700 1000) :rpy (list (deg2rad 60) 0 0))
                 (make-cascoords :pos (float-vector 8200 700 1100) :rpy (list (deg2rad 60) 0 0))
                 )
                :wrench-list
                (list
                 (float-vector 0 0 -1 0 0 0)
                 (float-vector 0 0 1 0 0 0)
                 )
                ))

(setq *IH-task*
      (instance task :init
                :name "IH"
                :attachable-plane-obj
                ;; 平面をキッチンの下側まで伸ばすと成功率が上がった
                (instance attachable-plane :init
                          :offset 8500
                          :min-angle-list (list -1250 800)
                          :max-angle-list (list 1600 2000)
                          :normal :x-)
                :coords-list
                (list
                 (make-cascoords :pos (float-vector 8200 -700 1200) :rpy (list 0 0 0))
                 (make-cascoords :pos (float-vector 8100 -600 1200) :rpy (list 0 0 0))
                 (make-cascoords :pos (float-vector 8200 -500 1200) :rpy (list 0 0 0))
                 )
                :wrench-list
                (list
                 (float-vector 0 0 -1 0 0 0)
                 (float-vector 0 0 -1 0 0 0)
                 (float-vector 0 0 -1 0 0 0)
                 )
                ))

(setq *table-task*
      (instance task :init
                :name "table"
                :attachable-plane-obj
                (instance attachable-plane :init
                          :offset 720
                          :min-angle-list (list 4350 -650)
                          :max-angle-list (list 4950 100)
                          :normal :z+)
                :coords-list
                (list
                 (make-cascoords :pos (float-vector 4450 -550 740) :rpy (list 0 0 0))
                 (make-cascoords :pos (float-vector 4450 0 740) :rpy (list 0 0 0))
                 (make-cascoords :pos (float-vector 4850 -550 740) :rpy (list 0 0 0))
                 (make-cascoords :pos (float-vector 4850 0 740) :rpy (list 0 0 0))
                 )
                :wrench-list
                (list
                 (float-vector 0 0 3 0 0 0)
                 (float-vector 0 0 3 0 0 0)
                 (float-vector 0 0 3 0 0 0)
                 (float-vector 0 0 3 0 0 0)
                 )
                ))

(setq *microwave-task*
      (instance task :init
                :name "microwave"
                :attachable-plane-obj
                (instance attachable-plane :init
                          :offset 1200
                          :min-angle-list (list 6020 1850)
                          :max-angle-list (list 6480 2230)
                          :normal :z+)
                :coords-list
                (list
                 (make-cascoords :pos (float-vector 6200 1800 1200) :rpy (list 0 0 0))
                 (make-cascoords :pos (float-vector 6200 1720 1120) :rpy (list 0 0 0))
                 )
                :wrench-list
                (list
                 (float-vector 0 6 0 0 0 0)
                 (float-vector 0 3 3 0 0 0)
                 )
                ))

(setq *fridge-task*
      (instance task :init
                :name "fridge"
                :attachable-plane-obj
                (instance attachable-plane :init
                          :offset 1850
                          :min-angle-list (list 6720 2000)
                          :max-angle-list (list 7300 2520)
                          :normal :z+)
                :coords-list
                (list
                 (make-cascoords :pos (float-vector 6750 2050 1850) :rpy (list 0 0 0))
                 (make-cascoords :pos (float-vector 6750 2450 1850) :rpy (list 0 0 0))
                 (make-cascoords :pos (float-vector 7200 2050 1850) :rpy (list 0 0 0))
                 ;; (make-cascoords :pos (float-vector 7200 2450 1850) :rpy (list 0 0 0))
                 )
                :wrench-list
                (list
                 (float-vector 0 0 2 0 0 0)
                 (float-vector 0 0 2 0 0 0)
                 (float-vector 0 0 2 0 0 0)
                 ;; (float-vector 0 0 3 0 0 0)
                 )
                ))

;; 入力するタスクの種類ごとに、異なるベース形状が出力される例。
;; 食事の準備：table + microwaveで三角形っぽい（力の種類が多い）？
;; 食事の準備：table + IHで、四角形
;; table単体でも、小さな四角形
;; fridge単体では、小さな三角形。頂点の内1つは吸着パッドと一致
;; fridge単体では、小さな三角形。頂点の内1つは吸着パッドと一致
;; IH単体では、吸着パッドを挟んで直線っぽい形状（なぜ？）
;; 洗い物：faucetのみで直線っぽい（力の方向が反対向きに2種類あるので、剛体脚が吸着パッドを挟んで直線上になっている）？
;; 電子レンジ専用動作：microwaveのみで一点っぽい（力の方向がほぼ1種類なので、短い直線。全ての剛体脚が1点に集まっている）
;; 拭き掃除：table + fridgeで、吸着パッドを挟んで直線っぽい形状（なぜ？）
;; 汎用動作：全部込みで四角形（台形）っぽい
;; メモ：タスクの数が増えると、*default-base-size-task-scale*の値を小さくしないと収束しない
(setq *tasks*
      (list
       *faucet-task*
       *IH-task*
       *table-task*
       *microwave-task*
       *fridge-task*
       ))
