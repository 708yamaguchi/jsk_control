(require "yamaguchi_6axis_arm_with_vacuum_base.l")
(defmethod yamaguchi_6axis_arm_with_vacuum_base-robot
  (:add-vacuum-base-coords
   ()
   (let ((base-link (find "base_link" (send self :links)
                          :test #'(lambda (x y) (equal x (send y :name)))))
         (rigid-leg-names (list :rigid-leg1-coords :rigid-leg2-coords :rigid-leg3-coords :rigid-leg4-coords))
         (rigid-leg-positions (list (float-vector 50 50 0) (float-vector -50 50 0) (float-vector -50 50 0) (float-vector -50 -50 0))))
     ;; TODO: 剛体脚の位置は直動関節で変えられるようにする
     ;; Define rigid legs fixed to base_link
     (dotimes (i (length rigid-leg-names))
       (send self :put (elt rigid-leg-names i)
             (make-cascoords
              :coords (send (send self :copy-worldcoords) :translate (elt rigid-leg-positions i))
              :parent base-link)))
     ;; Defind vacuum pad fixed to base_link
     (send self :put :vacuum-pad-coords
           (make-cascoords
            :coords (send self :copy-worldcoords)
            :parent base-link))
     )
   )
  (:end-coords
   ()
   (send self :rarm :end-coords))
  (:base-cube
   ()
   (let* ((x-size 160) (y-size 130) (z-size 80)
          (base-cube (make-cube x-size y-size z-size)))
     (send base-cube :newcoords (send self :copy-worldcoords))
     (send base-cube :translate (float-vector 0 0 (* 0.5 z-size)))
     (send base-cube :set-color (float-vector 0 1 0 0.3))
     base-cube))
  )

(defun yamaguchi-arm ()
  (let ((r (instance yamaguchi_6axis_arm_with_vacuum_base-robot :init)))
    (send r :add-vacuum-base-coords)
    r))

;; TODO: check robot weight
