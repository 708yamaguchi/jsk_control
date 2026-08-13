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
  (:size () _size)
  (:name () _name)
  (:attachable-plane-list () _attachable-plane-list)
  (:coords-list () _coords-list)
  (:wrench-list () _wrench-list)

  ;; coords-list や wrench-list を再利用し、新しい配置可能領域を設定したタスクを生成
  (:copy-with-attachable-plane
   (new-name new-attachable-plane-obj)
   (instance task :init
             :name new-name
             :attachable-plane-obj new-attachable-plane-obj
             :coords-list (send-all _coords-list :copy-worldcoords)
             :wrench-list _wrench-list))
  )

;; --- 既存タスク定義 ---

(setq *faucet-task*
      (instance task :init
                :name "faucet"
                :attachable-plane-obj
                (instance attachable-plane :init
                          :offset 900
                          :min-angle-list (list 7730 -270)
                          :max-angle-list (list 8300 570)
                          :normal :z+)
                :coords-list
                (list
                 (make-coords :pos (float-vector 8060 850 1150) :rpy (list (deg2rad 60) 0 0))
                 (make-coords :pos (float-vector 8060 850 1150) :rpy (list (deg2rad 60) 0 0))
                 )
                :wrench-list
                (list
                 (float-vector 0 0 -3 0 0 0)
                 (float-vector 0 0 3 0 0 0)
                 )
                ))

(setq *faucet-wall-task*
      (instance task :init
                :name "faucet-wall"
                :attachable-plane-obj
                (instance attachable-plane :init
                          :offset 8500
                          :min-angle-list (list -1250 800)
                          :max-angle-list (list 1600 2000)
                          :normal :x-)
                :coords-list
                (send-all (send *faucet-task* :coords-list) :copy-worldcoords)
                :wrench-list
                (send *faucet-task* :wrench-list)
                ))

(setq *IH-task*
      (instance task :init
                :name "IH"
                :attachable-plane-obj
                (instance attachable-plane :init
                          :offset 8500
                          :min-angle-list (list -1250 1000)
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

(setq *wipe-ih-wall-task*
      (instance task :init
                :name "wipe-IH-wall"
                :attachable-plane-obj
                (instance attachable-plane :init
                          :offset 8500
                          :min-angle-list (list -1250 800)
                          :max-angle-list (list 1600 2000)
                          :normal :x-)
                :coords-list
                (list
                 (make-coords :pos (float-vector 8450 -1100 1100) :rpy (list 0 0 0))
                 (make-coords :pos (float-vector 8450 -980 1500) :rpy (list 0 0 0))
                 (make-coords :pos (float-vector 8450 -190 1100) :rpy (list 0 0 0))
                 (make-coords :pos (float-vector 8450 -310 1500) :rpy (list 0 0 0))
                 )
                :wrench-list
                (list
                 (float-vector -1 0 0 0 0 0)
                 (float-vector -1 0 0 0 0 0)
                 (float-vector -1 0 0 0 0 0)
                 (float-vector -1 0 0 0 0 0)
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
                          :min-angle-list (list 6020 1900)
                          :max-angle-list (list 6340 2130)
                          :normal :z+)
                :coords-list
                (list
                 (make-coords :pos (float-vector 6200 1870 1200) :rpy (list 0 0 0))
                 (make-coords :pos (float-vector 6200 1650 1100) :rpy (list 0 0 0))
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

(setq *kitchen-task*
      (instance task :init
                :name "kitchen-task"
                :attachable-plane-obj
                (instance attachable-plane :init
                          :offset 8400
                          :min-angle-list (list -1250 900)
                          :max-angle-list (list 1600 2000)
                          :normal :x-)
                :coords-list
                (list
                 (make-coords :pos (float-vector 7870 -240 900) :rpy (list 0 0 0))
                 (make-coords :pos (float-vector 7870 540 900) :rpy (list 0 0 0))
                 (make-coords :pos (float-vector 8200 -240 900) :rpy (list 0 0 0))
                 (make-coords :pos (float-vector 8200 540 900) :rpy (list 0 0 0))
                 )
                :wrench-list
                (list
                 (float-vector 0 0 1 0 0 0)
                 (float-vector 0 0 1 0 0 0)
                 (float-vector 0 0 1 0 0 0)
                 (float-vector 0 0 1 0 0 0)
                 )
                ))

(setq *closet-task*
      (instance task :init
                :name "closet"
                :attachable-plane-obj
                (instance attachable-plane :init
                          :offset 2000
                          ;; :offset 1900
                          :min-angle-list (list 0 2100)
                          :max-angle-list (list 1700 2700)
                          :normal :y-)
                :coords-list
                (list
                 ;; (make-coords :pos (float-vector 2400 1900 700) :rpy (list 0 0 0))
                 ;; (make-coords :pos (float-vector 2400 1900 1500) :rpy (list 0 0 0))
                 (make-coords :pos (float-vector 2600 1900 700) :rpy (list 0 0 0))
                 (make-coords :pos (float-vector 2600 1900 1500) :rpy (list 0 0 0))
                 )
                :wrench-list
                (list
                 ;; (float-vector 0 -1 0 0 0 0)
                 ;; (float-vector 0 -1 0 0 0 0)
                 (float-vector 0 -1 0 0 0 0)
                 (float-vector 0 -1 0 0 0 0)
                 )
                ))



;; --- R8 用タスク定義 (各タスク専用の床面・配置領域を個別に指定) ---

(setq *r8-faucet-task*
      (send *faucet-task* :copy-with-attachable-plane
            "r8-faucet"
            (instance attachable-plane :init
                      :offset 10 ;; 視認できるようにちょっと上げる
                      :min-angle-list (list 6000 -1500)
                      :max-angle-list (list 7700 2000)
                      :normal :z+)))

(setq *r8-ih-task*
      (send *IH-task* :copy-with-attachable-plane
            "r8-IH"
            (instance attachable-plane :init
                      :offset 10 ;; 視認できるようにちょっと上げる
                      :min-angle-list (list 6000 -1500)
                      :max-angle-list (list 7700 2000)
                      :normal :z+)))

(setq *r8-wipe-ih-wall-task*
      (send *wipe-ih-wall-task* :copy-with-attachable-plane
            "r8-wipe-IH-wall" ;; 視認できるようにちょっと上げる
            (instance attachable-plane :init
                      :offset 10
                      :min-angle-list (list 6000 -1500)
                      :max-angle-list (list 8300 2000)
                      :normal :z+)))

(setq *r8-table-task*
      (send *table-task* :copy-with-attachable-plane
            "r8-table"
            (instance attachable-plane :init
                      :offset 0
                      :min-angle-list (list 3500 -1000)
                      :max-angle-list (list 5500 500)
                      :normal :z+)))

(setq *r8-microwave-task*
      (send *microwave-task* :copy-with-attachable-plane
            "r8-microwave"
            (instance attachable-plane :init
                      :offset 0
                      :min-angle-list (list 5500 1000)
                      :max-angle-list (list 7000 2500)
                      :normal :z+)))

(setq *r8-fridge-task*
      (send *fridge-task* :copy-with-attachable-plane
            "r8-fridge"
            (instance attachable-plane :init
                      :offset 0
                      :min-angle-list (list 6000 1500)
                      :max-angle-list (list 7800 3000)
                      :normal :z+)))

(setq *r8-kitchen-task*
      (send *kitchen-task* :copy-with-attachable-plane
            "r8-kitchen-task"
            (instance attachable-plane :init
                      :offset 10
                      :min-angle-list (list 6000 -1500)
                      :max-angle-list (list 7750 2000)
                      :normal :z+)))

(setq *r8-closet-task*
      (send *closet-task* :copy-with-attachable-plane
            "r8-closet"
            (instance attachable-plane :init
                      :offset 10 ;; 視認できるようにちょっと上げる
                      :min-angle-list (list 1500 500)
                      :max-angle-list (list 4000 1900)
                      :normal :z+)))

