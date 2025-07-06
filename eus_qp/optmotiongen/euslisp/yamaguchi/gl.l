;; https://github.com/euslisp/jskeus/issues/646
(defun change-scale-light
  (lightsource scale-light)
  (let ((default-ambient (float-vector 0.1 0.1 0.1 1.0))
        (default-diffuse (float-vector 1.0 1.0 1.0 1.0))
        (default-specular (float-vector 0.1 0.1 0.1 1.0)))
    (send lightsource :ambient (scale scale-light default-ambient))
    (send lightsource :diffuse (scale scale-light default-diffuse))
    (send lightsource :specular (scale scale-light default-specular))
    (send lightsource :opengl)))

(defun default-light ()
  (change-scale-light gl::*light0* 1.00)
  (change-scale-light gl::*light1* 0.7)
  (change-scale-light gl::*light2* 0.40)
  (change-scale-light gl::*light3* 0.25))

(defun room610-light ()
  (change-scale-light gl::*light0* 0.6)
  (change-scale-light gl::*light1* 0.1)
  (change-scale-light gl::*light2* 0.8)
  (change-scale-light gl::*light3* 0.3)
  )
