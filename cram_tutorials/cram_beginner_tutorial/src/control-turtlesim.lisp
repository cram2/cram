;;;
;;; Copyright (c) 2017, Lorenz Moesenlechner <moesenle@in.tum.de>
;;; Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;; Christopher Pollok <cpollok@uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :tut)

(defvar *color-value* (make-fluent :name :color-value) "current color under the turtle")
(defvar *turtle-pose* (make-fluent :name :turtle-pose) "current pose of turtle")

(defvar *color-sub* nil "color ROS subscriber")
(defvar *pose-sub* nil "pose ROS subscriber")
(defvar *cmd-vel-pub* nil "velocity commands ROS publisher")

(defvar *pen-srv* nil "name of ROS service for controlling the pen")

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
                                 "geometry_msgs/Twist"))
  (setf *pen-srv* (concatenate 'string "/" name "/set_pen")))
 
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

(defun call-set-pen (r g b width off)
  "Function to call the SetPen service."
  (call-service *pen-srv* 'turtlesim-srv:SetPen
                :r r
                :g g
                :b b
                :width width
                :off off))

(defun call-clear-background ()
  "Function to call the /clear service."
  (call-service "/clear" 'std_srvs-srv:Empty))
