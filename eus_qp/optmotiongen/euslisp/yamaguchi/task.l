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
   (dotimes (i _size)
     (setq _attachable-plane-list (append _attachable-plane-list (list attachable-plane-obj))))
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
                 (make-coords :pos (float-vector 8000 750 1000) :rpy (list (deg2rad 60) 0 0))
                 (make-coords :pos (float-vector 8100 650 1100) :rpy (list (deg2rad 60) 0 0))
                 )
                :wrench-list
                (list
                 (float-vector 0 0 -2 0 0 0)
                 (float-vector 0 0 2 0 0 0)
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
                 (make-coords :pos (float-vector 8200 -700 1200) :rpy (list 0 0 0))
                 (make-coords :pos (float-vector 8100 -600 1200) :rpy (list 0 0 0))
                 (make-coords :pos (float-vector 8200 -500 1200) :rpy (list 0 0 0))
                 )
                :wrench-list
                (list
                 (float-vector 0 1 -3 0 0 0)
                 (float-vector 1 0 -3 0 0 0)
                 (float-vector 0 -1 -3 0 0 0)
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
                 (make-coords :pos (float-vector 4450 -550 740) :rpy (list 0 0 0))
                 (make-coords :pos (float-vector 4450 0 740) :rpy (list 0 0 0))
                 (make-coords :pos (float-vector 4850 -550 740) :rpy (list 0 0 0))
                 (make-coords :pos (float-vector 4850 0 740) :rpy (list 0 0 0))
                 )
                :wrench-list
                (list
                 ;; (float-vector 0 0 3 0 0 0)
                 ;; (float-vector 0 0 3 0 0 0)
                 ;; (float-vector 0 0 3 0 0 0)
                 ;; (float-vector 0 0 3 0 0 0)
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
                          :min-angle-list (list 6020 1950)
                          :max-angle-list (list 6480 2130)
                          :normal :z+)
                :coords-list
                (list
                 (make-coords :pos (float-vector 6200 1800 1200) :rpy (list 0 0 0))
                 (make-coords :pos (float-vector 6200 1720 1120) :rpy (list 0 0 0))
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
                 (make-coords :pos (float-vector 6750 2050 1850) :rpy (list 0 0 0))
                 (make-coords :pos (float-vector 6750 2450 1850) :rpy (list 0 0 0))
                 (make-coords :pos (float-vector 7200 2050 1850) :rpy (list 0 0 0))
                 (make-coords :pos (float-vector 7200 2450 1850) :rpy (list 0 0 0))
                 )
                :wrench-list
                (list
                 (float-vector 0 0 2 0 0 0)
                 (float-vector 0 0 2 0 0 0)
                 (float-vector 0 0 2 0 0 0)
                 (float-vector 0 0 2 0 0 0)
                 )
                ))
