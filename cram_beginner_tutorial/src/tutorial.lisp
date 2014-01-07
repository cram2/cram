(in-package :tut)

(defvar *color-value* (make-fluent :name :color-value) "current color of turtle")
(defvar *turtle-pose* (make-fluent :name :turtle-pose) "current pose of turtle")

(defvar *color-sub* nil "color subscription client")
(defvar *pose-sub* nil "pose subscription client")
(defvar *cmd-vel-pub* nil "command velocity subscription client")

(defun init-ros-turtle (name)
  "subscribes to topics for a turtle and binds callbacks. `name' specifies the name of the turtle."
  (setf *color-sub* (subscribe (format nil "~a/color_sensor" name)
                               "turtlesim/Color"
                               #'color-cb))
  (setf *pose-sub* (subscribe (format nil "~a/pose" name)
                               "turtlesim/Pose"
                               #'pose-cb))
  (setf *cmd-vel-pub* (advertise (format nil "~a/cmd_vel" name)
                                 "geometry_msgs/Twist")))

(defun color-cb (msg)
  "Callback for color values"
  (setf (value *color-value*) msg))

(defun pose-cb (msg)
  "Callback for pose values"
  (setf (value *turtle-pose*) msg))

(defun send-vel-cmd (lin ang)
  "function to send velocity commands"
  (publish *cmd-vel-pub* (make-message "geometry_msgs/Twist"
                                      (linear) (make-msg "geometry_msgs/Vector3" (x) lin)
                                      (angular) (make-msg "geometry_msgs/Vector3" (z) ang) )))

(defun pose-msg->transform (msg)
  "returns a transform proxy that allows to transform into the frame given by x, y, and theta of msg."
  (with-fields (x y theta) msg
    (cl-transforms:make-transform
     (cl-transforms:make-3d-vector x y 0)
     (cl-transforms:axis-angle->quaternion
      (cl-transforms:make-3d-vector 0 0 1)
      theta))))

(defun relative-angle-to (goal pose)
  "Given a pose as 3d-pose msg and a goal as 3d vector, 
  calculate the angle by which the pose has to be turned to point toward the goal."
  (let ((diff-pose (cl-transforms:transform-point
                     (cl-transforms:transform-inv
                       (pose-msg->transform pose))
                     goal)))
    (atan
      (cl-transforms:y diff-pose)
      (cl-transforms:x diff-pose))))

(defun calculate-angular-cmd (goal &optional (ang-vel-factor 4))
  "Uses the current turtle pose and calculates the angular velocity
  command to turn towards the goal."
  (* ang-vel-factor
     (relative-angle-to goal (value *turtle-pose*))))

(def-cram-function move-to (goal &optional (distance-threshold 0.1))
  (let ((reached-fl (< (fl-funcall #'cl-transforms:v-dist
                                   (fl-funcall
                                    #'cl-transforms:translation
                                    (fl-funcall
                                     #'pose-msg->transform
                                     *turtle-pose*))
                                   goal)
                       distance-threshold)))
    (unwind-protect
         (pursue
           (wait-for reached-fl)
           (loop do
             (send-vel-cmd
               1.5
               (calculate-angular-cmd goal))
             (wait-duration 0.1)))
      (send-vel-cmd 0 0))))

(defstruct turtle-shape
  "represents an object in continuous space matching a symbolic description"
  radius
  edges)


(cram-reasoning:def-fact-group shape-actions (action-desig)

  ;; for each kind of shape, call make-turtle-shape with the right number of edges

  ;; triangle
  (<- (action-desig ?desig (shape ?act))
    (desig-prop ?desig (type shape))
    (desig-prop ?desig (shape triangle))
    (lisp-fun make-turtle-shape :radius 1 :edges 3  ?act))

  ;; square
  (<- (action-desig ?desig (shape ?act))
    (desig-prop ?desig (type shape))
    (desig-prop ?desig (shape square))
    (lisp-fun make-turtle-shape :radius 1 :edges 4  ?act))

  ;; pentagon
  (<- (action-desig ?desig (shape ?act))
    (desig-prop ?desig (type shape))
    (desig-prop ?desig (shape pentagon))
    (lisp-fun make-turtle-shape :radius 1 :edges 5  ?act))

  ;; hexagon
  (<- (action-desig ?desig (shape ?act))
    (desig-prop ?desig (type shape))
    (desig-prop ?desig (shape hexagon))
    (lisp-fun make-turtle-shape :radius 1 :edges 6  ?act)))

(cram-process-modules:def-process-module turtle-actuators (action-designator)
  (roslisp:ros-info (turtle-process-modules) "Turtle navigation invoked with action designator `~a'." action-designator)
  (destructuring-bind (cmd action-goal) (reference action-designator)
    (ecase cmd
      (shape
         (call-shape-action
          :edges (turtle-shape-edges action-goal)
          :radius (turtle-shape-radius action-goal))))))


(defmacro with-turtle-process-modules (&body body)
  `(cpm:with-process-modules-running
       (turtle-actuators turtle-navigation)
     ,@body))


         
                   
(def-fact-group turtle-actuators (matching-process-module
                                         available-process-module)

  (<- (matching-process-module ?designator turtle-actuators)
    (or (desig-prop ?designator (type shape))))

  (<- (available-process-module turtle-actuators)
    (symbol-value cram-projection:*projection-environment* nil)))




(defun start-tutorial ()
                (let ((turtle_name "turtle_1"))
                  (start-ros-node turtle_name)
                  (init-ros-turtle turtle_name)
))


;;;;;;;;REPL:
;;;(top-level (with-turtle-process-modules (with-designators (( my-desig (action '((type shape) (shape hexagon))))) (perform my-desig))))
;;;(top-level (with-turtle-process-modules (with-designators ((loc (location '((type turtle-position) (vpos top) (hpos center)))) (my-desig (action `((type navigation) (goal ,loc))))) (perform my-desig)))) 