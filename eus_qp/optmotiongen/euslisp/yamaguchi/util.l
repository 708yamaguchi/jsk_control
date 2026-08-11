(require "task.l")
(require "room610-model-list.l")

;; (visualize-tasks (list *ih-task* *faucet-task*))
(defun visualize-tasks (tasks)
  (let* (;; 可視化用オブジェクト。(send-all *task-arrow-list* :worldcoords)などはしない。
         (task-arrow-list
          (remove nil
                  (mapcar #'(lambda (coords wrench)
                              (if (equal (subseq wrench 0 3) (float-vector 0 0 0))
                                  nil
                                (make-single-arrow :length 200
                                                   :pos (send coords :worldpos)
                                                   :axis (subseq wrench 0 3)
                                                   :color #f(1 0.3 1))))
                          (flatten (send-all tasks :coords-list))
                          (flatten (send-all tasks :wrench-list))
                          )))
         (attachable-plane-list (flatten (send-all tasks :attachable-plane-list)))
         (additional-draw-objects
          (append *room610-model-list*
                  (send-all attachable-plane-list :object)
                  ;; (send-all *attachable-plane-list* :normal-object) ;; For debug
                  task-arrow-list)))
    ;; 2. Visualize
    (unless (boundp '*irtviewer*)
      (make-irtviewer))
    (send *irtviewer* :objects additional-draw-objects)
    (set-viewpoint "room610")
    (send *irtviewer* :draw-objects)
    additional-draw-objects)
  )


(defun print-sqp-result (sqp-opt args &key (debug-view t) (dissoc-root-virtual t))
  (let* ((sqp-config-task (send sqp-opt :config-task))
         (instant-config-task-list
          (if (derivedp sqp-config-task trajectory-configuration-task)
              (send sqp-config-task :instant-config-task-list)
            (list sqp-config-task)))
         collision)
    (format t "~%===== Result (in world coords) =====~%")
    (dotimes (i (length instant-config-task-list))
      (format t "task ~A: ~A~%" i (elt (gethash :task-name-list args) i))
      (let* ((config-task (elt instant-config-task-list i))
             (contact-attention-coords-list (send config-task :contact-attention-coords-list))
             (contact-target-coords-list (send config-task :contact-target-coords-list)))
        (format t "angle vector~%")
        (format t " ~A~%" (send (send config-task :robot-env) :robot :angle-vector))

        ;; 最終的な最適化の誤差
        (let* ((kin-val (send config-task :kinematics-task-value :update? nil))
               (e-kin (if kin-val (norm kin-val) nil))
               (eom-trans (send config-task :eom-trans-task-value :update? nil))
               (eom-rot (send config-task :eom-rot-task-value :update? nil))
               (base-val (if (and (fboundp 'rigid-legs-invariant?)
                                  (rigid-legs-invariant? (send config-task :invariant-joint-list)))
                             (send config-task :base-size-task-value :update? nil)
                           nil))
               (e-base (if base-val (norm base-val) nil))
               (total-val (send config-task :task-value :update? nil))
               (e-all (if total-val (norm total-val) nil)))
          (format t "Final Errors:~%")
          (format t "  e_pos: ~A~%" e-kin)
          (format t "  e_wrench(trans): ~A [N]~%" (if eom-trans (norm eom-trans) nil))
          (format t "  e_wrench(rot): ~A [Nm]~%" (if eom-rot (norm eom-rot) nil))
          (when e-base (format t "  e_base: ~A~%" e-base))
          (format t "  e: ~A~%" e-all))

        ;; Look at vacuum base shape
        (when (and debug-view (eq i 0)
                   (find-method (send (send config-task :robot-env) :robot) :base-convex-hull))
          (let* ((robot-env (copy-object (send config-task :robot-env)))
                 (v (send-lexpr (instantiate x::irtviewer) :create nil))
                 robot)
            (when dissoc-root-virtual
              (send robot-env :dissoc-root-virtual))
            (setq robot (send robot-env :robot :newcoords (make-coords)))
            (draw-base-convex-hull robot :v v)))
        (format t "Base worldcoords~%")
        (format t " ~A~%" (send (send config-task :robot-env) :robot :copy-worldcoords))
        ;; Print contact coords
        (format t "Contact coords~%")
        (dolist (contact-target-coords contact-target-coords-list)
          (format t " ~A~%" contact-target-coords))
        ;; Print wrench
        (dolist (force-type (list :force-list :moment-list :external-force-list :external-moment-list))
          (format t " ~A " force-type)
          (if (or (eq force-type :force-list) (eq force-type :external-force-list))
              (format t "[N]~%"))
          (if (or (eq force-type :moment-list) (eq force-type :external-moment-list))
              (format t "[Nm]~%"))
          (if (send config-task force-type)
              (progn
                (when (or (eq force-type :force-list) (eq force-type :moment-list))
                  (dotimes (j (length contact-attention-coords-list))
                    (format t "   ~A at ~A~%"
                            (elt (send config-task force-type) j)
                            (elt (send-all contact-attention-coords-list :name) j))))
                (when (or (eq force-type :external-force-list) (eq force-type :external-moment-list))
                  (dotimes (j (length (config-task . _external-coords-list)))
                    (format t "   ~A at ~A~%"
                            (elt (send config-task force-type) j)
                            (send (elt (config-task . _external-coords-list) j) :worldcoords))
                    )))
            (format t "   nil~%")))
        ;; Calc collision
        (let* ((robot-env (copy-object (send config-task :robot-env)))
               (robot (send robot-env :robot)))
          (dolist (link-pair (gethash :collision-avoidance-link-pair args))
            (when (eq (pqp-collision-check (car link-pair) (cadr link-pair)) 1)
              (setq collision t)
              (format t "Collision between ~A and ~A~%"
                      (send (car link-pair) :name)
                      (send (cadr link-pair) :name))))))
      (format t "~%"))
    (format t "only-kinematics? ~A~%" (send (car instant-config-task-list) :only-kinematics?))
    (format t "kin-scale ~A~%" (send (car instant-config-task-list) :kin-scale))
    (when (and (find-method (car instant-config-task-list) :base-size-scale)
               (send (car instant-config-task-list) :base-size-scale))
      (format t "base-size-scale ~A~%" (send (car instant-config-task-list) :base-size-scale)))
    (let ((robot (send (send (car instant-config-task-list) :robot-env) :robot)))
      (when (find-method robot :pad-diameter)
        (format t "Pad_diameter ~A~%" (send robot :pad-diameter)))
      (when (find-method robot :base-convex-hull)
        (let* ((robot-env (copy-object (send (car instant-config-task-list) :robot-env)))
               (robot-copy (send robot-env :robot)))
          (when dissoc-root-virtual
            (send robot-env :dissoc-root-virtual))
          (setq robot-copy (send robot-env :robot :newcoords (make-coords)))
          (format t "Base convex hull ~A~%" (send robot-copy :base-convex-hull))
          (format t "Base area [cm^2] ~A~%" (/ (send robot-copy :base-convex-hull-area) 100.0)))))
    (if collision
        (format t "Collision is found~%")
      (format t "Collision is not found~%"))
    )
  (format t "Status: ~A~%==================~%~%" (send sqp-opt :status))
  sqp-opt
  )

(defun get-args
    (robot tasks)
  (let* ((move-target (list (send robot :end-coords)
                            (send robot :get :rigid-leg0-coords)
                            (send robot :get :rigid-leg1-coords)
                            (send robot :get :rigid-leg2-coords)
                            (send robot :get :rigid-leg3-coords)
                            (send robot :get :vacuum-pad-coords)
                            (send robot :get :vacuum-pad-normal-coords)
                            ))
         (limb-size (1- (length move-target)))
         (task-coords-list (flatten (send-all tasks :coords-list)))
         (task-wrench-list (flatten (send-all tasks :wrench-list)))
         (task-target-coords (send-all task-coords-list :copy-worldcoords))
         (task-size (length task-coords-list))
         contact-target-coords-list
         contact-move-target-list
         (translation-axis-list (make-list task-size :initial-element
                                           (append (list t) (make-list limb-size :initial-element nil))))
         (rotation-axis-list (make-list task-size :initial-element
                                        (append (list nil) (make-list limb-size :initial-element nil))))
         (collision-avoidance-link-pair
          (append
           (mapcar #'(lambda (x) (list (send robot :base-cube) (elt (send robot :links) x)))
                   (list 2 3)) ;; base and arm
           (unless (derivedp robot r8-robot)
             (append
              (mapcar #'(lambda (x) (list (send robot :wrist-cube) (send (car x) :object)))
                      (send-all tasks :attachable-plane-list)) ;; wrist and plane
              (mapcar #'(lambda (x) (list (send robot :forearm-cube) (send (car x) :object)))
                      (send-all tasks :attachable-plane-list)) ;; forearm and plane
              (mapcar #'(lambda (x) (list (send robot :camera-cube) (send (car x) :object)))
                      (send-all tasks :attachable-plane-list)) ;; camera and plane
              ))
           ))
         sqp-opt target-coords-list move-target-list contact-constraint-list
         same-position-task-index temporary-invariant-joint-list adjacent-regular-scale-list
         (external-wrench-list (mapcar #'list task-wrench-list))
         (external-coords-list (make-list (length external-wrench-list) :initial-element (list (send robot :end-coords))))
         (args (make-hash-table :test #'equal))
         )
    (dotimes (i task-size)
      ;; メモ：各平面に固有の名前を付けないと、inverse-kinematics-statics-wrapper.lの以下のコードで除去される
      ;; (remove-duplicates (flatten target-coords-list) :test #'equal)
      (let ((attachable-plane-list (flatten (send-all tasks :attachable-plane-list)))
            task-face)
        (dotimes (j limb-size)
          (setq task-face (append task-face (list (send (elt attachable-plane-list i)
                                                        :face :margin 0
                                                        :name (format nil "~A,~A" i j)
                                                        )))))
        (setq target-coords-list
              (append target-coords-list
                      (list (append (list (elt task-target-coords i))
                                    task-face)))))
      (setq move-target-list (append move-target-list (list move-target)))
      (setq contact-constraint-list (append contact-constraint-list
                                            (list
                                             (list
                                              (instance rigid-leg-contact-constraint :init :name (format nil "rigid-leg 0, ~A" i) :size 2.5)
                                              (instance rigid-leg-contact-constraint :init :name (format nil "rigid-leg 1, ~A" i) :size 2.5)
                                              (instance rigid-leg-contact-constraint :init :name (format nil "rigid-leg 2, ~A" i) :size 2.5)
                                              (instance rigid-leg-contact-constraint :init :name (format nil "rigid-leg 3, ~A" i) :size 2.5)
                                              ;; 真空到達度-40kPaでは、直径80mmの吸着パッドが約-200Nを発揮可能。最大吸引力[N] = 直径[mm]^2 * -0.03125
                                              (instance vacuum-pad-contact-constraint :init :name (format nil "vacuum-pad-negative ~A" i)
                                                        :suction-force (* (send robot :pad-diameter) (send robot :pad-diameter) -0.03125))
                                              (instance rigid-leg-contact-constraint :init :name (format nil "vacuum-pad-normal ~A" i)
                                                        :size (* (send robot :pad-diameter) 0.5) :shape :octagon)
                                              )
                                             )))
      )
    (setq contact-target-coords-list (mapcar #'cdr target-coords-list))
    (setq contact-move-target-list (mapcar #'cdr move-target-list))
    ;; same-position-task-indexが(list i j)のとき、i, i+1, j, j+1番目のタスクが同じ場所で実行されるという意味
    (dotimes (i (- (length task-coords-list) 1))
      (when (equal (send (elt task-coords-list i) :name)
                   (send (elt task-coords-list (+ i 1)) :name))
        (setq same-position-task-index (append same-position-task-index (list i)))))
    (let (virtual-joint-list)
      (dolist (joint-type (list "linear-joint" "rotational-joint"))
        (dolist (axis (list "x" "y" "z"))
          (setq virtual-joint-list
                (append virtual-joint-list
                        (list (read-from-string
                               (format nil ":~A-~A-~A-~A"
                                       (send robot :name) "root-virtual" joint-type axis)))))))
      (dolist (j virtual-joint-list)
        (dolist (i same-position-task-index)
          (setq temporary-invariant-joint-list (append temporary-invariant-joint-list (list (list j i)))))))
    (setq adjacent-regular-scale-list (make-list (- task-size 1) :initial-element 0))
    (dolist (l same-position-task-index)
      (setf (elt adjacent-regular-scale-list l) 5e-4))
    (sethash :target-coords-list args target-coords-list)
    (sethash :move-target-list args move-target-list)
    (sethash :contact-target-coords-list args contact-target-coords-list)
    (sethash :contact-move-target-list args contact-move-target-list)
    (sethash :contact-constraint-list args contact-constraint-list)
    ;; メモ：external-wrenchは、手先が発揮しているレンチではなく、手先が受けているレンチ
    (sethash :external-wrench-list args external-wrench-list)
    (sethash :external-coords-list args external-coords-list)
    (sethash :translation-axis-list args translation-axis-list)
    (sethash :rotation-axis-list args rotation-axis-list)
    (sethash :collision-avoidance-link-pair args collision-avoidance-link-pair)
    (sethash :temporary-invariant-joint-list args temporary-invariant-joint-list)
    (sethash :adjacent-regular-scale-list args adjacent-regular-scale-list)
    (sethash :task-name-list args (flatten (mapcar #'(lambda(x) (make-list (send x :size) :initial-element (send x :name)))
                                                   tasks)))
    args
    ))

(defun get-r8-args
    (robot tasks)
  "R8ロボット用の引数抽出関数。接触・静力学考慮版"
  (let* ((base-coords-list (if (find-method robot :base-coords-list)
                               (send robot :base-coords-list)
                             (let ((bc (send robot :base-coords)))
                               (if (listp bc) bc (list bc)))))
         (move-target (append (list (send robot :end-coords)) base-coords-list))
         (limb-size (1- (length move-target)))
         (task-coords-list (flatten (send-all tasks :coords-list)))
         (task-wrench-list (flatten (send-all tasks :wrench-list)))
         (task-target-coords (send-all task-coords-list :copy-worldcoords))
         (task-size (length task-coords-list))
         (attachable-plane-list (flatten (send-all tasks :attachable-plane-list)))
         contact-target-coords-list
         contact-move-target-list
         (translation-axis-list (make-list task-size :initial-element
                                            (append (list t) (make-list limb-size :initial-element t))))
         (rotation-axis-list (make-list task-size :initial-element
                                         (append (list nil) (make-list limb-size :initial-element t))))
         (collision-avoidance-link-pair nil)
         sqp-opt target-coords-list move-target-list contact-constraint-list
         same-position-task-index temporary-invariant-joint-list adjacent-regular-scale-list
         (external-wrench-list (mapcar #'list task-wrench-list))
         (external-coords-list (make-list (length external-wrench-list) :initial-element (list (send robot :end-coords))))
         (args (make-hash-table :test #'equal)))

    (dotimes (i task-size)
      (let (task-face)
        (dotimes (j limb-size)
          (setq task-face (append task-face (list (send (elt attachable-plane-list i)
                                                        :face :margin 0
                                                        :name (format nil "~A,~A" i j))))))
        (setq target-coords-list
              (append target-coords-list
                      (list (append (list (elt task-target-coords i))
                                    task-face)))))
      (setq move-target-list (append move-target-list (list move-target)))
      (setq contact-constraint-list (append contact-constraint-list
                                             (list
                                              (let (c-list)
                                                (dotimes (k limb-size)
                                                  (setq c-list (append c-list (list (instance rigid-leg-contact-constraint :init :name (format nil "rigid-leg ~A, ~A" k i) :size 10)))))
                                                c-list)))))

    (setq contact-target-coords-list (mapcar #'cdr target-coords-list))
    (setq contact-move-target-list (mapcar #'cdr move-target-list))

    (dotimes (i (- (length task-coords-list) 1))
      (when (equal (send (elt task-coords-list i) :name)
                   (send (elt task-coords-list (+ i 1)) :name))
        (setq same-position-task-index (append same-position-task-index (list i)))))
    (let (virtual-joint-list)
      (dolist (joint-type (list "linear-joint" "rotational-joint"))
        (dolist (axis (list "x" "y" "z"))
          (setq virtual-joint-list
                (append virtual-joint-list
                        (list (read-from-string
                               (format nil ":~A-~A-~A-~A"
                                       (send robot :name) "root-virtual" joint-type axis)))))))
      (dolist (j virtual-joint-list)
        (dolist (i same-position-task-index)
          (setq temporary-invariant-joint-list (append temporary-invariant-joint-list (list (list j i)))))))
    (setq adjacent-regular-scale-list (make-list (- task-size 1) :initial-element 0))
    (dolist (l same-position-task-index)
      (setf (elt adjacent-regular-scale-list l) 5e-4))

    (sethash :target-coords-list args target-coords-list)
    (sethash :move-target-list args move-target-list)
    (sethash :contact-target-coords-list args contact-target-coords-list)
    (sethash :contact-move-target-list args contact-move-target-list)
    (sethash :contact-constraint-list args contact-constraint-list)
    (sethash :external-wrench-list args external-wrench-list)
    (sethash :external-coords-list args external-coords-list)
    (sethash :translation-axis-list args translation-axis-list)
    (sethash :rotation-axis-list args rotation-axis-list)
    (sethash :collision-avoidance-link-pair args collision-avoidance-link-pair)
    (sethash :temporary-invariant-joint-list args temporary-invariant-joint-list)
    (sethash :adjacent-regular-scale-list args adjacent-regular-scale-list)
    (sethash :task-name-list args (flatten (mapcar #'(lambda (x)
                                                       (make-list (send x :size) :initial-element (send x :name)))
                                                   tasks)))
    args))
