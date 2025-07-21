;; Room 610

(load "models/room610-scene.l")
(load "models/toshiba-microwave-oven-object.l")

(unless (boundp '*room610*)
  (setq *room610* (room610)))

(unless (boundp '*toshiba-microwave-oven*)
  (setq *toshiba-microwave-oven* (toshiba-microwave-oven))
  (send *toshiba-microwave-oven* :newcoords
        (make-coords :pos (float-vector 6300 2200 800) :rpy (list (deg2rad -90) 0 0)))

(unless (boundp '*room610-model-list*)
  (setq *room610-model-list*
        (append
         (send *room610* :objects)
         (list *toshiba-microwave-oven*))))

(defun set-room610-viewpoint ()
  (send *irtviewer* :viewer :viewing :look
        #f(-8636.77 -8927.11 10455.5)
        #f(6740.47 1058.99 706.424)
        #f(0.0 0.0 1.0)))

(defun set-viewpoint (name)
  (cond ((equal name "room610")
         (send *irtviewer* :viewer :viewing :look
               #f(-8636.77 -8927.11 10455.5)
               #f(6740.47 1058.99 706.424)
               #f(0.0 0.0 1.0)))
        ((equal name "fridge")
         (send *irtviewer* :viewer :viewing :look
               #f(5438.79 -481.969 4157.96)
               #f(6253.24 873.508 3091.33)
               #f(0.0 0.0 1.0)))
        ((equal name "IH")
         (send *irtviewer* :viewer :viewing :look
               #f(6665.19 -3115.46 2621.31)
               #f(7495.54 -1884.42 1831.78)
               #f(0.0 0.0 1.0)))
        ((equal name "faucet")
         (send *irtviewer* :viewer :viewing :look
               #f(5484.0 -938.376 2915.31)
               #f(7143.14 58.5374 1705.81)
               #f(0.0 0.0 1.0)))
        ((equal name "microwave")
         (send *irtviewer* :viewer :viewing :look
               #f(3664.02 -200.18 3179.07)
               (float-vector 6200 1800 1200)
               #f(0.0 0.0 1.0)))
        ((equal name "table")
         (send *irtviewer* :viewer :viewing :look
               #f(453.714 -3414.34 3758.35)
               #f(3025.62 -1476.27 1973.26)
               #f(0.0 0.0 1.0)))
        )
  (send *irtviewer* :draw-objects))

(defun check-viewpoints ()
  (dolist (name (list "table" "IH" "faucet" "fridge" "microwave" "room610"))
    (set-viewpoint name)
    (unix:sleep 2)))

;; Arrow

(require "models/arrow-object.l")
(require "models/single-arrow-object.l")

(defun make-single-arrow (&key (pos (float-vector 0 0 0)) (axis (float-vector 0 1 0))
                               (length 200.0)
                               (color (float-vector 1 0 0)))
  (let* ((ar (single-arrow))
         (size-scale (/ length 200.0))
         (bodies (send (car (send ar :links)) :bodies))
         (faces-list (send-all bodies :faces))
         (vertices-list (mapcar #'(lambda (x) (send-all x :vertices)) faces-list))
         scaled-vertices-list
         scaled-faces-list
         scaled-bodies
         scaled-arrow
         )
    (setq scaled-vertices-list
          (mapcar #'(lambda (x) (mapcar #'(lambda (y) (mapcar #'(lambda (z) (scale size-scale z)) y)) x))
                  vertices-list))
    (setq scaled-faces-list
          (mapcar #'(lambda (x) (mapcar #'(lambda (y) (instance face :init :vertices y)) x))
                  scaled-vertices-list))
    (setq scaled-bodies
          (mapcar #'(lambda (x) (instance faceset :init :faces x)) scaled-faces-list))
    (send-all scaled-bodies :set-color color)
    (send-all scaled-bodies :transform (new-arrow-coords pos axis))
    (setq scaled-arrow
          (instance bodyset-link :init (make-cascoords)
                    :bodies scaled-bodies
                    :name :scaled-arrow-bodyset2
                    :weight 1
                    :centroid (float-vector 0.0 0.0 0.0)
                    :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))
    scaled-arrow
  ))

(defun new-arrow-coords (pos axis)
  (let ((axis-y (normalize-vector axis))
        axis-x axis-z
        (mat (make-matrix 3 3)))
    (when (equal axis (float-vector 0 1 0))
      (return-from new-arrow-coords (make-coords :pos pos)))
    (setq axis-x (v* axis-y (float-vector 0 1 0)))
    (setq axis-z (v* axis-x axis-y))
    (setf (matrix-column mat 0) axis-x)
    (setf (matrix-column mat 1) axis-y)
    (setf (matrix-column mat 2) axis-z)
    (make-coords :pos pos :rot mat)
    ))
