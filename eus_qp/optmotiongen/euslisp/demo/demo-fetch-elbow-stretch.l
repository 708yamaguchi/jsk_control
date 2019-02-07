(load "package://fetcheus/fetch.l")
(load "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-wrapper.l")

(defun demo-fetch-elbow-stretch (&key (stretch-elbow t) (pre-process-func) (post-process-func) &allow-other-keys)
  (fetch)
  (objects (list *fetch*))
  (send *fetch* :reset-pose)
  (send *fetch* :torso :waist-z :joint-angle 250)
  (send* *fetch*
         :rarm
         :inverse-kinematics-optmotiongen
         (make-coords :pos (float-vector 600 0 950) :rpy (float-vector 0 pi/2 0))
         :use-torso nil
         :pre-process-func pre-process-func
         :post-process-func post-process-func
         (if stretch-elbow
             (list :posture-joint-list (list (nth 5 (send *fetch* :rarm :joint-list)))
                   :posture-joint-angle-list (list 0)
                   :target-posture-scale 1e-2))
         )
  (objects (list *fetch*)))
