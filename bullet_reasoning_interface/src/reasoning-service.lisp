(in-package :reas-inf)

(defvar *transform-listener*)

(defmacro out-info (&rest args)
  `(ros-info (bullet-reasoning-interface) ,@args))

(defmacro out-error (&rest args)
  `(ros-error (bullet-reasoning-interface) ,@args))

(defmacro out-debug (&rest args)
  `(ros-debug (bullet-reasoning-interface) ,@args))

(defun set-transform-listener ()
      (out-info "Setting *transform-listener*")
    (defparameter *transform-listener* (make-instance 'cl-tf2:buffer-client)))

(defun init-interface ()
  (if (eq (node-status) :RUNNING)
      (out-info "No need to start another node.")
      (start-ros-node "bullet-reasoning-interface"))
  (spatial-relations-demo::start-ros-and-bullet)
  (set-transform-listener)
  (out-info "Initialized bullet-reasoning-interface."))

(defun start-service ()
  (out-info "Starting service.")
  (interaction-server))

(defun get-object-type (type)
  (let ((bowl-const (get-interaction-constant ':BOWL))
        (nesquik-const (get-interaction-constant ':NESQUIK))
        (mondamin-const (get-interaction-constant ':MONDAMIN))
        (fruit-apple-const (get-interaction-constant ':FRUIT_APPLE))
        (fruit-orange-const (get-interaction-constant ':FRUIT_ORANGE))
        (sugar-const (get-interaction-constant ':SUGAR)))
    (cond
      ((eq type bowl-const) 'spatial-relations-demo::bowl)
      ((eq type nesquik-const) 'spatial-relations-demo::cereal)
      ((eq type mondamin-const) 'spatial-relations-demo::mondamin)
      ((eq type fruit-apple-const) 'spatial-relations-demo::apple)
      ((eq type fruit-orange-const) 'spatial-relations-demo::orange)
      ((eq type sugar-const) 'spatial-relations-demo::sugar-box))))

;;; pose ist im frame designators-ros:*fixed-frame*

(defun color-msg-to-list (msg)
  (with-fields (r g b) msg
    `(,r ,g ,b)))

(defun make-color-msg (r g b)
  (make-msg "std_msgs/ColorRGBA" :r r :g g :b b))

(defun make-pose-stamped-msg (frame position-x position-y position-z
                              &optional (rotation-x 0) (rotation-y 0) (rotation-z 0) (rotation-w 1))
  (make-msg "geometry_msgs/PoseStamped"
            (:frame_id :header) frame
            (:x :position :pose) position-x
            (:y :position :pose) position-y
            (:z :position :pose) position-z
            (:x :orientation :pose) rotation-x
            (:y :orientation :pose) rotation-y
            (:z :orientation :pose) rotation-z
            (:w :orientation :pose) rotation-w))
            

(defun spawn-object (name type pose-stamped-msg color-msg)
  (out-info "spawn-object()")
  (if (object-exists name)
      (progn
        (out-error "Object ~a already exists!" name)
        nil)
      (let ((type-symbol (get-object-type type))
            (name-symbol (intern name)))
        (out-info "type-symbol: ~a" type-symbol)
        (out-info "name-symbol: ~a" name-symbol)
        (with-fields ((from-frame (frame_id header))) pose-stamped-msg
          (let* ((pose-stamped (cl-tf2:from-msg pose-stamped-msg))
                 (pose-stamped-fixed (if (string= (cl-tf2:unslash-frame from-frame) (cl-tf2:unslash-frame designators-ros:*fixed-frame*))
                                         pose-stamped
                                         (cl-tf2:transform-pose
                                          *transform-listener*
                                          :pose pose-stamped
                                          :target-frame designators-ros:*fixed-frame*)))
                 (color (color-msg-to-list color-msg)))
            (spatial-relations-demo::spawn-object name-symbol type-symbol pose-stamped-fixed color)))
        (object-exists name))))

(defun move-object (name pose-stamped-msg &optional (world btr:*current-bullet-world*))
  (if (not (object-exists name world))
      (progn
        (out-error "Trying to move non existent object ~a!" name)
        nil)
      (with-fields ((from-frame (frame_id header))
                    (pose-msg pose)) pose-stamped-msg
        (out-info "pose-msg: ~a" pose-msg)
        (let* ((name-symbol (intern name))
               (pose (cl-tf2:from-msg pose-msg))
               (pose-stamped (cl-tf2:from-msg pose-stamped-msg))
               (pose-stamped-fixed (if (string= (cl-tf2:unslash-frame from-frame) (cl-tf2:unslash-frame designators-ros:*fixed-frame*))
                                       pose-stamped
                                       (cl-tf2:transform-pose
                                        *transform-listener*
                                        :pose pose-stamped
                                        :target-frame designators-ros:*fixed-frame*))))
          (prolog `(assert (object-pose ,world ,name-symbol ,pose-stamped-fixed)))
          (poses-equal-p pose (get-object-pose name world) 0.01 0.01)))))

(defun object-exists (name &optional (world btr:*current-bullet-world*))
  (not (null (force-ll (prolog `(object ,world ,(intern name)))))))

(defun get-object-instance (name &optional (world btr:*current-bullet-world*))
  (let ((result (cram-utilities:var-value '?instance
                            (car (prolog `(%object ,world ,(intern name) ?instance))))))
    (if (eq result '?instance)
        nil
        result)))

(defun get-object-pose (name &optional (world btr:*current-bullet-world*))
  (let ((result (get-object-instance name world)))
    (if result
        (pose result))))

(defun get-object-pose-stamped (name &optional (world btr:*current-bullet-world*))
  (let ((result (get-object-pose name world)))
    (if result
        (cl-tf:make-pose-stamped designators-ros:*fixed-frame* (ros-time) (cl-tf:origin result) (cl-tf:orientation result)))))

(defun remove-obj (name &optional (world btr:*current-bullet-world*))
  (unless (object-exists name world)
    (out-error "Trying to remove non existent object ~a" name))
  (let ((name-symbol (intern name)))
    (prolog `(retract (object ,world ,name-symbol))))
  (not (object-exists name world)))
  
(defun get-interaction-constant (name)
  (let ((ret (roslisp-msg-protocol:symbol-code 'bullet_reasoning_interface-srv:<interaction-request> name)))
    (out-debug "interaction-constant ~a: ~a" name ret)
    ret))

(defun is-in-solution (variable-name symbol solution)
  "Returns `t' if the solution `solution' contains a tuple with `variable-name' as its car nad `symbol' as its cdr."
  (find t (mapcar (lambda (list) (list-contains-tuple `(,variable-name . ,symbol) list)) solution)))

(defun list-contains-tuple (tuple list)
  "Returns `t' if the list `list' contains a tuple with the same car and cdr values as `tuple'. `list' is expected to be a list of tuples."
  (let ((car-val (car list))
        (cdr-val (cdr list)))
    (if (tuple-contains (car car-val) (cdr car-val) tuple)
        t
        (if cdr-val
            (list-contains-tuple tuple cdr-val)))))

(defun tuple-contains (car-val cdr-val tuple)
  "Returns `t' if the tuple `tuple' contains `car-val' in its car and `cdr-val' in its cdr."
  (and (eq (car tuple) car-val) (eq (cdr tuple) cdr-val)))

(defun is-visible (object &optional (world btr:*current-bullet-world*))
  (not (null (prolog `(visible ,world cram-pr2-knowledge::pr2 ,object)))))
;;  (is-in-solution '?objects object (force-ll (prolog `(visible ,world cram-pr2-knowledge::pr2 ?objects)))))

(defun is-stable-world (&optional (world btr:*current-bullet-world*))
  (prolog `(stable ,world)))

(defun is-stable-object (object &optional (world btr:*current-bullet-world*))
  (not (null (force-ll (prolog `(stable ,world ,object))))))

(defun simulate-world (duration &optional (world btr:*current-bullet-world*))
  (prolog `(simulate ,world ,duration)))
  

(def-service-callback bullet_reasoning_interface-srv:Interaction (operations objectIds objectTypes objectColors poses simulate duration)
  (out-info "Incoming Request:")
  (out-info "operations: ~a" operations)
  (out-info "objectIds: ~a" objectIds)
  (out-info "objectTypes: ~a" objectTypes)
  (out-debug "objectColors: ~a" objectColors)
  (out-debug "poses: ~a" poses)
  (out-info "simulate: ~a" simulate)
  (out-info "duration: ~a" duration)
  (if (not (= (length operations) (length objectIds) (length objectTypes) (length objectColors) (length poses)))
      (progn
        (out-error "Invalid input. I need arrays of the same size.")
        (make-response :errorLevel (get-interaction-constant ':ERROR)))
      (let ((spawn-object-const (get-interaction-constant ':SPAWN_OBJECT))
            (move-object-const (get-interaction-constant ':MOVE_OBJECT))
            (remove-object-const (get-interaction-constant ':REMOVE_OBJECT))
            (success-const (get-interaction-constant ':SUCCESS))
            (failed-const (get-interaction-constant ':FAILED))
            (error-const (get-interaction-constant ':ERROR))
            (unhandled-value-const (get-interaction-constant ':UNHANDLED_VALUE))
            (confirmation (make-array (length operations) :fill-pointer 0)))
          (loop for i from 0 to (- (length operations) 1) do
            (let ((operation (elt operations i))
                  (objectId (elt objectIds i))
                  (objectType (elt objectTypes i))
                  (objectColor (elt objectColors i))
                  (pose (elt poses i)))
              (cond
                ((eq operation spawn-object-const)
                 (if (spawn-object objectId objectType pose objectColor)
                     (vector-push success-const confirmation)
                     (vector-push failed-const confirmation)))
                ((eq operation move-object-const)
                 (out-info "Move!")
                 (vector-push failed-const confirmation))
                ((eq operation remove-object-const)
                 (out-info "Remove!")
                 (vector-push failed-const confirmation))
                (t
                 (out-error "Unhandled operation.")
                 (vector-push unhandled-value-const confirmation))))
                (out-info "---------------------------------"))
        (make-response :errorLevel (get-interaction-constant ':SUCCESS) :confirmation confirmation))))

(defun interaction-server ()
  (with-ros-node ("interaction_server" :spin t)
    (set-transform-listener)
    (register-service "interaction" 'bullet_reasoning_interface-srv:Interaction)
    (out-info "Ready to receive requests.")))
