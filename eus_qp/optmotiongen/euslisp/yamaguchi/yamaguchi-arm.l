(require "yamaguchi_6axis_arm_with_vacuum_base.l")
(defmethod yamaguchi_6axis_arm_with_vacuum_base-robot
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
  (instance yamaguchi_6axis_arm_with_vacuum_base-robot :init))
