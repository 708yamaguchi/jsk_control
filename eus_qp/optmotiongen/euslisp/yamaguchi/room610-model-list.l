;; Room 610

(load "models/room610-scene.l")
(load "models/toshiba-microwave-oven-object.l")
(load "models/room602-square-600-closet-object.l")
(load "faucet.l")

(require "gl.l")

(unless (boundp '*room610*)
  (setq *room610* (room610)))

;; 除外したいオブジェクト名のリスト（将来消したい物体名をここに追加できます）
(defvar *room610-remove-object-names*
  (list "room610-x-leg-desk" "room610-sweeper" "room610-bears-panel"
        ))

;; *room610* から指定されたオブジェクトを削除（同名物体が複数あっても警告なしで全削除）
(when (boundp '*room610*)
  (dolist (name *room610-remove-object-names*)
    (let ((targets (send *room610* :find-object name)))
      (dolist (target targets)
        (send *room610* :dissoc target))
      (setf (slot *room610* scene-model 'objs)
            (set-difference (slot *room610* scene-model 'objs) targets)))))

;; *room610* 内の洗濯機、冷蔵庫などの配置変更
(let ((laundry (find-if #'(lambda (o) (substringp "room610-laundry-machine" (send o :name))) (send *room610* :objects)))
      (fridge (find-if #'(lambda (o) (substringp "room610-fridge" (send o :name))) (send *room610* :objects)))
      (tv1 (find-if #'(lambda (o) (substringp "room610-tv1" (send o :name))) (send *room610* :objects)))
      (audio-shelf (find-if #'(lambda (o) (substringp "room610-audio_shelf" (send o :name))) (send *room610* :objects)))
      )
  (when laundry
    (send laundry :newcoords
          (make-coords :pos (float-vector 8550 2200 0) :rpy (list pi 0 0))))
  (when fridge
    (send fridge :newcoords
          (make-coords :pos (float-vector 4650 2250 0) :rpy (list -pi/2 0 0))))
  (when tv1
    (send tv1 :newcoords
          (make-coords :pos (float-vector 3500 2200 700) :rpy (list -pi/2 0 0))))
  (when audio-shelf
    (send audio-shelf :newcoords
          (make-coords :pos (float-vector 3500 2200 0) :rpy (list -pi/2 0 0))))
  )

;; 電子レンジの追加
(unless (boundp '*toshiba-microwave-oven*)
  (setq *toshiba-microwave-oven* (toshiba-microwave-oven))
  (send *toshiba-microwave-oven* :newcoords
        (make-coords :pos (float-vector 6300 2150 800) :rpy (list (deg2rad -90) 0 0))))

;; 蛇口の追加
(unless (boundp '*faucet*)
  (setq *faucet* (make-faucet))
  (send *faucet* :newcoords
        (make-coords :pos (float-vector 8200 770 900) :rpy (list (/ pi 3) 0 0))))

;; クローゼットの追加
(unless (boundp '*room602-square-600-closet*)
  (setq *room602-square-600-closet* (room602-square-600-closet))
  (send *room602-square-600-closet* :newcoords
        (make-coords :pos (float-vector 2400 2300 0) :rpy (list (/ -pi 2) 0 0))))

(unless (boundp '*room610-model-list*)
  (setq *room610-model-list*
        (append
         (send *room610* :objects)
         (list *toshiba-microwave-oven*)
         (list *faucet*)
         (list *room602-square-600-closet*))))

(defun set-room610-viewpoint ()
  (send *irtviewer* :viewer :viewing :look
        #f(-8636.77 -8927.11 10455.5)
        #f(6740.47 1058.99 706.424)
        #f(0.0 0.0 1.0)))

;; (format t "To set current viewpoint, use the following command:~%~A~%"
;;         `(send *irtviewer* :viewer :viewing :look
;;                ,(send *irtviewer* :viewpoint)
;;                ,(send *irtviewer* :viewtarget)
;;                #f(0 0 1)))
(defun set-viewpoint (name)
  (room610-light)
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
               #f(1415.94 -3432.64 4507.37)
               #f(2106.83 -3153.5 4175.6)
               #f(0.0 0.0 1.0)))
        ((equal name "faucet")
         (send *irtviewer* :viewer :viewing :look
               #f(2944.55 -2108.16 4206.12)
               #f(3003.36 -2074.2 4170.01)
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
