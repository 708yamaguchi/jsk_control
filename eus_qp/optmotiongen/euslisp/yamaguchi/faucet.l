(defun make-handle (rad)
  (let* ((div 16) handle-points handle)
    (dotimes (i div)
      (let ((p (scale rad
                      (float-vector (cos (* i (/ 2pi div)))
                                    (sin (* i (/ 2pi div)))
                                    0))))
        (setq handle-points
              (append handle-points
                      (list (if (< i (/ div 2))
                                (v+ p (float-vector 0 100 0))
                              (v+ p (float-vector 0 -100 0))))))))
    (setq handle (make-prism handle-points 10))))


(defun make-faucet ()
  (let* ((rad 25)
         (base (make-cylinder rad 150))
         (handle (make-handle rad))
         (pipe (make-cylinder rad 250))
         (pipe- (make-cube (* 2 (+ rad 4)) 50 20))
         (tip (make-cylinder (+ rad 4) 60)))
    (send handle :rotate (/ pi 6.0) :x)
    (send handle :translate (float-vector 0 80 200) :world)
    (send handle :set-color (float-vector 0.99 0.99 0.99))
    (send pipe :rotate (/ pi -3.0) :x)
    (send pipe :translate (float-vector 0 15 100) :world)
    (send pipe- :translate (float-vector 0 210 240) :world)
    (send pipe- :set-color (float-vector 1 0 0 0.5)) ;; for debug
    (setq pipe (body- pipe pipe-))
    (send tip :translate (float-vector 0 220 170) :world)

    (send base :assoc handle)
    (send base :assoc pipe)
    (send base :assoc tip)
    (instance bodyset :init (make-cascoords)
              :bodies (list base handle pipe tip))
    ))
