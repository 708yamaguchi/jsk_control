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
                          :min-angle-list (list 7800 -250)
                          :max-angle-list (list 8300 600)
                          :normal :z+)
                :coords-list
                (list
                 (make-cascoords :pos (float-vector 8200 700 1000) :rpy (list (deg2rad 60) 0 0))
                 ;; (make-cascoords :pos (float-vector 8300 700 1100) :rpy (list (deg2rad 60) 0 0))
                 ;; (make-cascoords :pos (float-vector 7800 600 1000) :rpy (list (deg2rad 60) 0 0))
                 )
                :wrench-list
                (list
                 (float-vector 0 0 -1 0 0 0)
                 ;; (float-vector 0 0 -1 0 0 0)
                 ;; (float-vector 0 0 -1 0 0 0)
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
                 (make-cascoords :pos (float-vector 8200 -600 1000) :rpy (list 0 0 0))
                 (make-cascoords :pos (float-vector 8200 -500 1000) :rpy (list 0 0 0))
                 )
                :wrench-list
                (list
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
                          :min-angle-list (list 4300 -700)
                          :max-angle-list (list 5000 150)
                          :normal :z+)
                :coords-list
                (list
                 (make-cascoords :pos (float-vector 4700 -50 740) :rpy (list 0 0 0))
                 (make-cascoords :pos (float-vector 4400 -150 740) :rpy (list 0 0 0))
                 (make-cascoords :pos (float-vector 4400 -450 740) :rpy (list 0 0 0))
                 (make-cascoords :pos (float-vector 4700 -550 740) :rpy (list 0 0 0))
                 )
                :wrench-list
                (list
                 (float-vector 0 0 1 0 0 0)
                 (float-vector 0 0 1 0 0 0)
                 (float-vector 0 0 1 0 0 0)
                 (float-vector 0 0 1 0 0 0)
                 )
                ))

(setq *microwave-task*
      (instance task :init
                :name "microwave"
                :attachable-plane-obj
                (instance attachable-plane :init
                          :offset 1200
                          :min-angle-list (list 6050 1850)
                          :max-angle-list (list 6450 2200)
                          :normal :z+)
                :coords-list
                (list
                 (make-cascoords :pos (float-vector 6200 1800 1200) :rpy (list 0 0 0))
                 )
                :wrench-list
                (list
                 (float-vector 0 0 -1 0 0 0)
                 )
                ))

(setq *fridge-task*
      (instance task :init
                :name "fridge"
                :attachable-plane-obj
                (instance attachable-plane :init
                          :offset 1850
                          :min-angle-list (list 6720 2010)
                          :max-angle-list (list 7300 2520)
                          :normal :z+)
                :coords-list
                (list
                 (make-cascoords :pos (float-vector 6900 2400 1850) :rpy (list 0 0 0))
                 ;; (make-cascoords :pos (float-vector 6850 2325 1850) :rpy (list 0 0 0))
                 (make-cascoords :pos (float-vector 6800 2250 1850) :rpy (list 0 0 0))
                 ;; (make-cascoords :pos (float-vector 6850 2175 1850) :rpy (list 0 0 0))
                 (make-cascoords :pos (float-vector 6900 2100 1850) :rpy (list 0 0 0))
                 )
                :wrench-list
                (list
                 (float-vector 0 0 1 0 0 0)
                 ;; (float-vector 0 0 1 0 0 0)
                 (float-vector 0 0 1 0 0 0)
                 ;; (float-vector 0 0 1 0 0 0)
                 (float-vector 0 0 1 0 0 0)
                 )
                ))

(setq *tasks*
      (list
       *faucet-task*
       *IH-task*
       *table-task*
       ;; *microwave-task*
       ;; *fridge-task*
       ))
