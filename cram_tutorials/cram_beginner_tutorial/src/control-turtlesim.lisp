(in-package :tut)

(defvar *color-value* (make-fluent :name :color-value) "current color under the turtle")
(defvar *turtle-pose* (make-fluent :name :turtle-pose) "current pose of turtle")

(defvar *color-sub* nil "color ROS subscriber")
(defvar *pose-sub* nil "pose ROS subscriber")
(defvar *cmd-vel-pub* nil "velocity commands ROS publisher")

(defun init-ros-turtle (name)
  "Subscribes to topics for a turtle and binds callbacks.
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
  "Function to send velocity commands."
  (publish *cmd-vel-pub*
           ;; short syntax:
           ;; (make-message "geometry_msgs/Twist" (:x :linear) lin (:z :angular) ang)
           ;; more understandable syntax:
           (make-message "geometry_msgs/Twist"
                         :linear (make-msg "geometry_msgs/Vector3" :x lin)
                         :angular (make-msg "geometry_msgs/Vector3" :z ang))))
