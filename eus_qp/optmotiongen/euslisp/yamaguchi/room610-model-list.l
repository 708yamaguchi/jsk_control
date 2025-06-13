(load "models/room610-scene.l")
(load "models/toshiba-microwave-oven-object.l")

(unless (boundp '*room610*)
  (setq *room610* (room610)))

(unless (boundp '*toshiba-microwave-oven*)
  (setq *toshiba-microwave-oven* (toshiba-microwave-oven))
  (send *toshiba-microwave-oven* :newcoords
        (make-coords :pos (float-vector 6300 2100 800) :rpy (list (deg2rad -90) 0 0)))

(unless (boundp '*room610-model-list*)
  (setq *room610-model-list*
        (append
         (send *room610* :objects)
         (list *toshiba-microwave-oven*))))

(defun set-room610-viewpoint ()
  (send *irtviewer* :viewer :viewing :look
        #f(-4408.27 -4047.55 7061.9)
        #f(4884.01 -103.211 1918.36)
        #f(0.0 0.0 1.0)))
