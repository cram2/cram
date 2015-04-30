(in-package :reas-inf)

(defvar *camera* :camera)
(defvar *camera-view* :camera-view)

(defun exists-any-object (&optional (world btr:*current-bullet-world*))
  "Returns `t' if there exists any object in the world `world', `NIL' otherwise."
  (out-debug "exists-any-object()")
  (not (null (force-ll (prolog `(household-object-type ,world ?objects ?types))))))

(defun object-exists (name &optional (world btr:*current-bullet-world*))
  "Returns `t' if the object `name' exists in the world `world', `NIL' otherwise."
  (out-info "object-exists()")
  (not (null (force-ll (prolog `(object ,world ,(make-keyword name)))))))

(defun get-object-instance (name &optional (world btr:*current-bullet-world*))
  "Returns the instance of object `name' in the world `world'."
  (out-debug "get-object-instance()")
  (out-debug "Getting object instance from object ~a in world ~a" name world)
  (let ((result (cram-utilities:var-value '?instance
                            (car (prolog `(%object ,world ,(make-keyword name) ?instance))))))
    (if (eq result '?instance)
        nil
        result)))

(defun get-robot-instance (&key (world btr:*current-bullet-world*) (name 'cram-pr2-knowledge::pr2))
  (cdr (car (car (force-ll (prolog `(%object ,world ,name ?rob)))))))

(defun get-robot-pose (&key (world btr:*current-bullet-world*) (name 'cram-pr2-knowledge::pr2))
  (pose (get-robot-instance :world world :name name)))

(defun get-robot-pose-stamped (&key (world btr:*current-bullet-world*) (name 'cram-pr2-knowledge::pr2))
  (let ((result (get-robot-pose :world world :name name)))
    (cl-tf:make-pose-stamped  designators-ros:*fixed-frame* (ros-time) (cl-tf:origin result) (cl-tf:orientation result))))

(defun get-object-pose (name &key (world btr:*current-bullet-world*) simulate-duration copy)
  "Returns the pose of object `name' in the world `world'."
  (out-debug "Getting pose for object ~a in world ~a" name world)
  (if simulate-duration
      (get-object-pose name :world (simulate-world simulate-duration :world world :copy copy))
      (let ((result (get-object-instance name world)))
        (if result
            (pose result)))))

(defun get-camera-pose (&optional (world btr:*current-bullet-world*))
  (cdaar (prolog `(object-pose ,world ,*camera* ?pose))))

(defun get-object-dimensions (name &optional (world btr:*current-bullet-world*))
  (let ((box (get-object-bounding-box name world)))
    (when box
      (cl-bullet:bounding-box-dimensions box))))

(defun get-object-bounding-box (name &optional (world btr:*current-bullet-world*))
  (out-debug "get-object-bounding-box() name: ~a" name)
  (let ((dimensions (get-box-dimensions name world)))
    (when (null dimensions)
        (setf dimensions (get-mesh-dimensions name world)))
    (when (null dimensions)
        (setf dimensions (get-sphere-dimensions name world)))
    (when dimensions
      (cl-bullet:make-bounding-box
       :center (get-object-pose name :world world)
       :dimensions dimensions))))

(defun get-box-dimensions (name &optional (world btr:*current-bullet-world*))
  (with-failure-handling ((simple-error (e)
                            (declare (ignore e))
                            (return)))
    (cl-tf:v* 
     (cl-bullet:half-extents (cl-bullet:collision-shape (car (rigid-bodies (get-object-instance name world)))))
     2.0)))

(defun get-mesh-dimensions (name &optional (world btr:*current-bullet-world*))
  (with-failure-handling ((simple-error (e)
                            (declare (ignore e))
                            (return)))
    (get-points-dimensions (get-mesh-points name world))))

(defun get-sphere-dimensions (name &optional (world btr:*current-bullet-world*))
  (with-failure-handling ((simple-error (e)
                            (declare (ignore e))
                            (return))
                          (simple-type-error (e)
                            (declare (ignore e))
                            (return)))
    (let ((d (* 2 (get-sphere-radius name world))))
      (cl-tf:make-3d-vector d d d))))

(defun get-sphere-radius (name &optional (world btr:*current-bullet-world*))
  (out-debug "Getting radius from object ~a." name)
  (with-failure-handling ((simple-error (e)
                            (declare (ignore e))
                            (return)))
    (cl-bullet:radius (cl-bullet:collision-shape (car (rigid-bodies (get-object-instance name world)))))))

(defun get-mesh-points (name &optional (world btr:*current-bullet-world*))
  (with-failure-handling ((simple-error (e)
                            (declare (ignore e))
                            (return)))
    (cl-bullet:points (cl-bullet:collision-shape (car (rigid-bodies (get-object-instance name world)))))))

(defun get-points-dimensions (points)
  (when points
    (let* ((first-point (elt points 0))
           (min-x (cl-tf:x first-point))
           (min-y (cl-tf:y first-point))
           (min-z (cl-tf:z first-point))
           (max-x (cl-tf:x first-point))
           (max-y (cl-tf:y first-point))
           (max-z (cl-tf:z first-point)))
      (loop for point across (subseq points 1)
            do (let ((new-x (cl-tf:x point))
                     (new-y (cl-tf:y point))
                     (new-z (cl-tf:z point)))
                 (if (> new-x max-x)
                     (setf max-x new-x)
                     (when (< new-x min-x)
                       (setf min-x new-x)))
                 (if (> new-y max-y)
                     (setf max-y new-y)
                     (when (< new-y min-y)
                       (setf min-y new-y)))
                 (if (> new-z max-z)
                     (setf max-z new-z)
                     (when (< new-z min-z)
                       (setf min-z new-z)))))
      (cl-tf:make-3d-vector (- max-x min-x) (- max-y min-y) (- max-z min-z)))))
        
(defun get-object-pose-stamped (name &optional (world btr:*current-bullet-world*))
  "Returns the stamped pose of object `name' in the world `world'."
  (let ((result (get-object-pose name :world world)))
    (when result
        (cl-tf:make-pose-stamped designators-ros:*fixed-frame* (ros-time) (cl-tf:origin result) (cl-tf:orientation result)))))

(defun is-visible (object &key (world btr:*current-bullet-world*) camera-pose simulate-duration copy)
  "Returns `t' if the object `name' is visible from the robot's viewpoint in the world `world', `NIL' otherwise.
If `camera-pose' is given, this pose is used for evaluation instead of the robot's viewpoint.
If `simulate-duration' is given, the world will be simulated for this duration before avaluating.
If `copy' is `t', the simulation and the evaluation will be executed on a copy of `world'."
  (out-info "Checking if object ~a is visible" object)
  (if simulate-duration
      (is-visible object :world (simulate-world simulate-duration :world world :copy copy) :camera-pose camera-pose)
      (if camera-pose
          (progn
            (remove-camera)
            (let ((result
                    (not (null (prolog `(visible-from ,world ,camera-pose ,(make-keyword object)))))))
              (update-camera-pose camera-pose world)
              result))
          (not (null (prolog `(visible ,world cram-pr2-knowledge::pr2 ,(make-keyword object))))))))

(defun is-stable-world (&key (world btr:*current-bullet-world*) simulate-duration copy)
  "Returns `t' if world `world' is stable, `NIL' otherwise."
  (out-info "Checking if world ~a is stable" world)
  (let ((result
          (if simulate-duration
              (is-stable-world :world (simulate-world simulate-duration :world world :copy copy))
              (not (null (force-ll (prolog `(stable-household ,world))))))))
    (get-elapsed-time)
    result))
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
        (not (null result)))))

(defun object-has-contact-with-kitchen (object &key (world btr:*current-bullet-world*) simulate-duration copy)
  (out-info "object-has-contact-with-kitchen()")
  (let ((result
          (if simulate-duration
              (object-has-contact-with-kitchen  object :world (simulate-world simulate-duration :world world :copy copy))
              (is-in-solution '?objects 'spatial-relations-demo::my-kitchen
                              (force-ll (prolog `(contact ,world ,(make-keyword object) ?objects)))))))
    (get-elapsed-time)
    result))

(defun have-collision (object-1 object-2  &key (world btr:*current-bullet-world*) simulate-duration copy)
  "Returns `t' if `object-1' and `object-2' are colliding in world `world', `NIL' otherwise.
If `simulate-duration' is given, the world will be simulated for this duration before evaluating.
If `copy' is `t', the simulation and the evaluation will be executed on a copy of `world'."
  (out-info "Checking if object ~a and object ~a are colliding." object-1 object-2)
  (if (and (not (object-exists object-1 world)) (not (object-exists object-2 world)))
      (out-error "Looking if non existent object ~a has collision with non existent object ~a." object-1 object-2)
      (if (not (object-exists object-1 world))
          (out-error "Looking if non existent object ~a has collision with object ~a." object-1 object-2)
          (if (not (object-exists object-2 world))
              (out-error "Looking if object ~a has collision with non existent object ~a." object-1 object-2))))
  (if simulate-duration
      (have-collision object-1 object-2 :world (simulate-world simulate-duration :world world :copy copy))
      (not (null (force-ll (prolog `(contact ,world ,(make-keyword object-1) ,(make-keyword object-2))))))))

(defun object-get-collisions (object &key (world btr:*current-bullet-world*) simulate-duration copy (elem-type :symbol) (list-type :vector))
  "Returns all collisions for `object' in world `world'.
If `elem-type' is `:symbol', the values are returned as symbols.
If `elem-type' is `:string', the values are returned as strings.
If `list-type' is `:vector', the result is returned as vector.
If `list-type' is `:list', the result is returned as list.
If `simulate-duration' is given, the world will be simulated for this duration before evaluating.
If `copy' is `t', the simulation and the evaluation will be executed on a copy of `world'."
  (out-info "Getting collisions for object ~a." object)
  (unless (object-exists object world)
    (out-error "Looking for collisions for non existent object ~a." object))
  (if simulate-duration
      (object-get-collisions object :world (simulate-world simulate-duration :world world :copy copy))
      (let ((result (get-all-x-from-solution '?objects (force-ll (prolog `(and
                                                                           (contact ,world ,(make-keyword object) ?objects)
                                                                           (household-object-type ,world ?objects ?type)))))))
        (when (eq elem-type :string)
          (setf result (mapcar #'resolve-keyword result)))
        (when (eq list-type :vector)
          (setf result (list-to-vector result)))
        (get-elapsed-time)
        result)))
          

(defun get-stable-objects  (&key (world btr:*current-bullet-world*) simulate-duration copy)
  "Returns a lsit of all stable objects in the world `world'.
If `simulate-duration' is given, the world will be simulated for this duration before avaluating.
If `copy' is `t', the simulation and the evaluation will be executed on a copy of `world'."
  (out-info "Getting stable objects from world ~a" world)
  (let ((result
          (if simulate-duration
              (get-stable-objects :world (simulate-world simulate-duration :world world :copy copy))
              (get-all-x-from-solution '?object (force-ll (prolog `(and (household-object-type ,world ?object ?type)
                                                                        (stable ,world ?object))))))))
    (get-elapsed-time)
    result))

(defun get-visible-objects  (&key (world btr:*current-bullet-world*) camera-pose simulate-duration copy)
  "Returns a lsit of all objects that are visible from the robot's viewpoint in the world `world'.
If `camera-pose' is given, this pose is used for evaluation instead of the robot's viewpoint.
If `simulate-duration' is given, the world will be simulated for this duration before avaluating.
If `copy' is `t', the simulation and the evaluation will be executed on a copy of `world'."
  (out-info "Getting visible objects from world ~a" world)
  (if simulate-duration
      (get-visible-objects :world (simulate-world simulate-duration :world world :copy copy) :camera-pose camera-pose)
      (get-all-x-from-solution '?object
                               (let ((result
                                       (if camera-pose
                                           (progn
                                             (remove-camera)
                                             (let ((result
                                                     (force-ll (prolog `(and
                                                                         (household-object-type ,world ?object ?type)
                                                                         (visible-from ,world ,camera-pose ?object))))))
                                               (update-camera-pose camera-pose world)
                                               result))
                                           (force-ll (prolog `(and (household-object-type ,world ?object ?type)
                                                                   (visible ,world ?robot ?object)))))))
                                 (get-elapsed-time)
                                 result))))

(defun get-all-objects (&optional (world btr:*current-bullet-world*))
  "Returns a list of all objects in the world `world'."
  (get-all-x-from-solution
   '?obj
   (force-ll (prolog `(and (household-object-type ,world ?obj ?type) (object ,world ?obj))))))
