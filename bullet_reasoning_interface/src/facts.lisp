(in-package :reas-inf)

(defun exists-any-object (&optional (world btr:*current-bullet-world*))
  "Returns `t' if there exists any object in the world `world', `NIL' otherwise."
  (not (null (force-ll (prolog `(household-object-type ,world ?objects ?types))))))

(defun object-exists (name &optional (world btr:*current-bullet-world*))
  "Returns `t' if the object `name' exists in the world `world', `NIL' otherwise."
  (not (null (force-ll (prolog `(object ,world ,(make-keyword name)))))))

(defun get-object-instance (name &optional (world btr:*current-bullet-world*))
  "Returns the instance of object `name' in the world `world'."
  (out-info "Getting object instance from object ~a in world ~a" name world)
  (let ((result (cram-utilities:var-value '?instance
                            (car (prolog `(%object ,world ,(make-keyword name) ?instance))))))
    (if (eq result '?instance)
        nil
        result)))

(defun get-object-pose (name &key (world btr:*current-bullet-world*) simulate-duration copy)
  "Returns the pose of object `name' in the world `world'."
  (out-info "Getting pose for object ~a in world ~a" name world)
  (if simulate-duration
      (get-object-pose name :world (simulate-world simulate-duration :world world :copy copy))
      (let ((result (get-object-instance name world)))
        (if result
            (pose result)))))

(defun get-object-pose-stamped (name &optional (world btr:*current-bullet-world*))
  "Returns the stamped pose of object `name' in the world `world'."
  (let ((result (get-object-pose name :world world)))
    (when result
        (cl-tf:make-pose-stamped designators-ros:*fixed-frame* (ros-time) (cl-tf:origin result) (cl-tf:orientation result)))))

(defun is-visible (object &key (world btr:*current-bullet-world*) simulate-duration copy)
  "Returns `t' if the object `name' is visible from the robot's viewpoint in the world `world', `NIL' otherwise."
  (out-info "Checking if object ~a is visible" object)
  (if simulate-duration
      (is-visible object :world (simulate-world simulate-duration :world world :copy copy))
      (not (null (prolog `(visible ,world cram-pr2-knowledge::pr2 ,(make-keyword object)))))))

(defun is-stable-world (&key (world btr:*current-bullet-world*) simulate-duration copy)
    "Returns `t' if world `world' is stable, `NIL' otherwise."
  (out-info "Checking if world ~a is stable" world)
  (if simulate-duration
      (is-stable-world :world (simulate-world simulate-duration :world world :copy copy))
      (not (null (force-ll (prolog `(stable-household ,world)))))))
      ;;(prolog `(stable ,world))))

(defun is-stable-object (object &key (world btr:*current-bullet-world*) simulate-duration copy)
  "Returns `t' if the object `object' is stable in the world `world', `NIL' otherwise.
If `simulate-duration' is given, the world will be simulated for this duration before evaluating.
If `copy' is `t', the simulation and the evaluation will be executed on a copy of `world'."
  (out-info "Checking if object ~a in world ~a is stable. simulate-duration: ~a, copy: ~a" object world simulate-duration copy)
  (unless (object-exists object world)
    (out-error "Looking if non existent object ~a is stable!" object))
  (if simulate-duration
      (is-stable-object object :world (simulate-world simulate-duration :world world :copy copy))
      (let ((result (force-ll (prolog `(stable ,world ,(make-keyword object))))))
        (format t "result: ~a" result)
        (not (null result)))))

(defun get-stable-objects  (&key (world btr:*current-bullet-world*) simulate-duration copy)
  "Returns a lsit of all stable objects in the world `world'.
If `simulate-duration' is given, the world will be simulated for this duration before avaluating.
If `copy' is `t', the simulation and the evaluation will be executed on a copy of `world'."
  (out-info "Getting stable objects from world ~a" world)
  (if simulate-duration
      (get-stable-objects :world (simulate-world simulate-duration :world world :copy copy))
      (get-all-x-from-solution '?object (force-ll (prolog `(and (household-object-type ,world ?object ?type)
                                                                (stable ,world ?object)))))))

(defun get-visible-objects  (&key (world btr:*current-bullet-world*) simulate-duration copy)
  "Returns a lsit of all objects that are visible from the robot's viewpoint in the world `world'.
If `simulate-duration' is given, the world will be simulated for this duration before avaluating.
If `copy' is `t', the simulation and the evaluation will be executed on a copy of `world'."
  (out-info "Getting visible objects from world ~a" world)
  (if simulate-duration
      (get-visible-objects :world (simulate-world simulate-duration :world world :copy copy))
      (get-all-x-from-solution '?object (force-ll (prolog `(and (household-object-type ,world ?object ?type)
                                                                (visible ,world ?robot ?object)))))))

(defun get-all-objects (&optional (world btr:*current-bullet-world*))
  "Returns a list of all objects in the world `world'."
  (get-all-x-from-solution
   '?obj
   (force-ll (prolog `(and (household-object-type ,world ?obj ?type) (object ,world ?obj))))))
