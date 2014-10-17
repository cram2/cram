;;;; This tutorial implements some basic functionality to control a turtle
;;;; in turtlesim using CRAM.

(in-package :tut)

;;; ROS infrastructure for monitoring the color of the ground
;;; of turtlesim and the pose of the turtle and for commanding
;;; its velocity.

(defvar *color-value* (make-fluent :name :color-value) "current color under turtle")
(defvar *turtle-pose* (make-fluent :name :turtle-pose) "current pose of turtle")

(defvar *color-sub* nil "color ROS subscriber")
(defvar *pose-sub* nil "pose ROS subscriber")
(defvar *cmd-vel-pub* nil "velocity commands ROS publisher")

(defun init-ros-turtle (name)
  "subscribes to topics for a turtle and binds callbacks.
`name' specifies the name of the turtle."
  (setf *color-sub* (subscribe (format nil "~a/color_sensor" name)
                               "turtlesim/Color"
                               #'color-cb))
  (setf *pose-sub* (subscribe (format nil "~a/pose" name)
                               "turtlesim/Pose"
                               #'pose-cb))
  (setf *cmd-vel-pub* (advertise (format nil "~a/cmd_vel" name)
                                 "geometry_msgs/Twist")))

(defun color-cb (msg)
  "Callback for color values. Called by the color topic subscriber."
  (setf (value *color-value*) msg))

(defun pose-cb (msg)
  "Callback for pose values. Called by the pose topic subscriber."
  (setf (value *turtle-pose*) msg))

(defun send-vel-cmd (lin ang)
  "Function to send velocity commands"
  (if (equal nil *cmd-vel-pub*)
    (roslisp:ros-error "cram-beginner-tutorial" "Cannot send velocity command because velocity publisher is nil. Are you sure you initialized the turtle?")
    (publish *cmd-vel-pub* (make-message "geometry_msgs/Twist"
                                       :linear (make-msg "geometry_msgs/Vector3"
                                                         :x lin)
                                       :angular (make-msg "geometry_msgs/Vector3"
                                                          :z ang)))))

;;; CRAM function MOVE to send the turtle to a specified coordinate
;;; in turtlesim and stop it when the coordinate has been reached
;;; (CRAM's function PURSUE is used for that) and its utility functions

(defun pose-msg->transform (msg)
  "Returns a transform proxy that allows to transform into the frame
given by x, y, and theta of msg."
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
  (if (or (equal nil *turtle-pose*) (equal nil (value *turtle-pose*)))
    (roslisp:ros-error "cram-beginner-tutorial" "Cannot get current pose because turtle-pose is nil. Are you sure you initialized the turtle?")
    (* ang-vel-factor
       (relative-angle-to goal (value *turtle-pose*)))))

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

;;; CRAM process modules for the turtle to move in geometric shape trajectories

(defstruct turtle-shape
  "Represents a geometric object in continuous space matching a symbolic description"
  radius
  edges)

(cram-reasoning:def-fact-group shape-actions (action-desig)
  ;; for each kind of shape, call make-turtle-shape with the right number of edges

  ;; triangle
  (<- (action-desig ?desig (shape ?action))
    (desig-prop ?desig (type shape))
    (desig-prop ?desig (shape triangle))
    (lisp-fun desig-prop-value ?desig radius ?radius)
    (lisp-fun make-turtle-shape :radius ?radius :edges 3  ?action))

  ;; square
  (<- (action-desig ?desig (shape ?action))
    (desig-prop ?desig (type shape))
    (desig-prop ?desig (shape square))
    (lisp-fun desig-prop-value ?desig radius ?radius)
    (lisp-fun make-turtle-shape :radius ?radius :edges 4  ?action))

  ;; pentagon
  (<- (action-desig ?desig (shape ?action))
    (desig-prop ?desig (type shape))
    (desig-prop ?desig (shape pentagon))
    (lisp-fun desig-prop-value ?desig radius ?radius)
    (lisp-fun make-turtle-shape :radius ?radius :edges 5  ?action))

  ;; hexagon
  (<- (action-desig ?desig (shape ?action))
    (desig-prop ?desig (type shape))
    (desig-prop ?desig (shape hexagon))
    (lisp-fun desig-prop-value ?desig radius ?radius)
    (lisp-fun make-turtle-shape :radius ?radius :edges 6  ?action)))

(cram-process-modules:def-process-module turtle-actuators (action-designator)
  (roslisp:ros-info (turtle-process-modules)
                    "Turtle navigation invoked with action designator `~a'."
                    action-designator)
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
    (desig-prop ?designator (type shape)))

  (<- (available-process-module turtle-actuators)
    (symbol-value cram-projection:*projection-environment* nil)))

(defun start-tutorial ()
  "First make sure roscore, the turtlesim and the actionlib server are running:
$ roscore
$ rosrun turtlesim turtlesim_node
$ rosrun turtle_actionlib shape_server"
  (let ((turtle-name "turtle1"))
    (start-ros-node turtle-name)
    (init-ros-turtle turtle-name)
    (top-level
      (with-turtle-process-modules
        (cpm:process-module-alias :manipulation 'turtle-actuators)
          (cram-language-designator-support:with-designators
            ((trajectory (action '((type shape) (shape hexagon))))
              (loc (location '((type navigation) (vpos top) (hpos center)))))
            (cpm:pm-execute :manipulation trajectory))))))


;;;;;;;;REPL:
;;;(top-level (with-turtle-process-modules (with-designators (( my-desig (action '((type shape) (shape hexagon))))) (perform my-desig))))
;;;(top-level (with-turtle-process-modules (with-designators ((loc (location '((type turtle-position) (vpos top) (hpos center)))) (my-desig (action `((type navigation) (goal ,loc))))) (perform my-desig)))) 
