(in-package :pr2-em)

;;; SIMULATION MANIPULATION

(defun move-joint (name angle)
  (format T "Move joint ~a to angle ~a.~%" name angle)
  (btr:set-robot-state-from-joints
   `((,name ,angle))
   (btr:object btr:*current-bullet-world* :kitchen)))

(defun open-container-joint (container-type)
  (format T "Open container ~a.~%" container-type)
  (move-joint (cdr (assoc container-type *container-joints*))
              (cdr (assoc container-type *container-angles*))))

(defun close-container-joint (container-type)
  (format T "Close container ~a.~%" container-type)
  (move-joint (cdr (assoc container-type *container-joints*))
              0))

(defun open-all ()
  (dolist (c *container*) (open-container-joint c)))

(defun close-all ()
  (dolist (c *container*) (close-container-joint c)))

;;; TESTING

(defun get-opening-desig (&optional (?name 'sink_area_left_upper_drawer_main))
  (let ((?object (get-container-desig ?name)))
    (an action
        (type opening)
        (object ?object))))

(defun get-container-desig (&optional (?name 'sink_area_left_upper_drawer_main))
  (let* ((name-str (roslisp-utilities:rosify-underscores-lisp-name ?name))
         (urdf-pose (get-urdf-link-pose name-str)))
    (let* ((?pose (cl-tf:transform-pose-stamped cram-tf:*transformer*
                                                :target-frame "base_footprint"
                                                :pose (cl-tf:pose->pose-stamped "map" 0 urdf-pose)))
           (?transform (cl-tf:make-transform-stamped "base_footprint" name-str
                                                     (cl-tf:stamp ?pose)
                                                     (cl-tf:origin ?pose)
                                                     (cl-tf:orientation ?pose))))
      (an object
          (type container)
          (name ?name)
          (part-of kitchen)
          (pose ((pose ?pose) (transform ?transform)))
          ))))

(defun test-open ()
  (cram-pr2-projection:with-simulated-robot
    (let ((?desig (get-opening-desig 'sink_area_left_upper_drawer_main)))
      (exe:perform ?desig))))

(defun test ()
  (cram-pr2-projection:with-simulated-robot
    (let ((?object (get-container-desig 'sink_area_left_upper_drawer_main)))
          (exe:perform (an action (type driving-and-opening) (object ?object))))))

(defun move-pr2 (x y)
  (cram-pr2-projection:with-simulated-robot
        (let ((?goal (cl-transforms-stamped:make-pose-stamped
                    "map"
                    0.0
                    (cl-tf:make-3d-vector x y 0.0)
                    (cl-tf:make-identity-rotation))))
        (exe:perform (a motion (type going) (target (a location (pose ?goal))))))))
