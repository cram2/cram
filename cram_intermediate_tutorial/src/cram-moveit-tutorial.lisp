;;; Copyright (c) 2015, Mihai Pomarlan <blandc@cs.uni-bremen.de>
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
;;;     * Neither the name of the Universitaet Bremen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(in-package :tuti)

(defvar *pose-mid* nil)
(defvar *pose-cube* nil)
(defvar *pose-right* nil)
(defvar *pose-right-msg* nil)
(defvar *cube-mesh* nil)
(defvar *start-robot-state* nil)
(defvar *planned-trajectory* nil)

(defvar *tf2-tb* nil)
(defvar *tf2* nil)

(defun init-cram-moveit-tutorial ()
  (start-ros-node "tutorial-client")
  (setf *tf2-tb* (cl-tf2:make-transform-broadcaster))
  (setf *tf2* (make-instance 'cl-tf2:buffer-client))
  (moveit:init-moveit-bridge)
  (setf *pose-mid*
         (cl-tf-datatypes:make-pose-stamped
           "/odom_combined"
           0.0
           (cl-transforms:make-3d-vector 0.67 0.0 0.94)
           (cl-transforms:make-quaternion 0.0 0.0 0.0 1.0)))
  (setf *pose-cube*
         (cl-tf-datatypes:make-pose-stamped
           "/odom_combined"
           0.0
           (cl-transforms:make-3d-vector 0.67 -0.20 0.94)
           (cl-transforms:make-quaternion 0.0 0.0 0.0 1.0)))
  (setf *pose-right*
         (cl-tf-datatypes:make-pose-stamped
           "/odom_combined"
           0.0
           (cl-transforms:make-3d-vector 0.67 -0.45 0.94)
           (cl-transforms:make-quaternion 0.0 0.0 0.0 1.0)))
  (setf *pose-right-msg*
        (roslisp:make-msg "geometry_msgs/Pose"
                          :position (roslisp:make-msg "geometry_msgs/Point" :x 0.67 :y -0.45 :z 0.94)
                          :orientation (roslisp:make-msg "geometry_msgs/Quaternion" :x 0.0 :y 0.0 :z 0.0 :w 1.0)))
  (setf *cube-mesh*
         (roslisp:make-msg
           "shape_msgs/Mesh"
           :triangles (make-array 12 :initial-contents 
                                     (list 
                                       (roslisp:make-msg "shape_msgs/MeshTriangle" :vertex_indices (make-array 3 :initial-contents '(0 2 1)))
                                       (roslisp:make-msg "shape_msgs/MeshTriangle" :vertex_indices (make-array 3 :initial-contents '(2 0 3)))
                                       (roslisp:make-msg "shape_msgs/MeshTriangle" :vertex_indices (make-array 3 :initial-contents '(4 5 6)))
                                       (roslisp:make-msg "shape_msgs/MeshTriangle" :vertex_indices (make-array 3 :initial-contents '(6 7 4)))
                                       (roslisp:make-msg "shape_msgs/MeshTriangle" :vertex_indices (make-array 3 :initial-contents '(0 1 5)))
                                       (roslisp:make-msg "shape_msgs/MeshTriangle" :vertex_indices (make-array 3 :initial-contents '(1 2 6)))
                                       (roslisp:make-msg "shape_msgs/MeshTriangle" :vertex_indices (make-array 3 :initial-contents '(2 3 7)))
                                       (roslisp:make-msg "shape_msgs/MeshTriangle" :vertex_indices (make-array 3 :initial-contents '(3 0 4)))
                                       (roslisp:make-msg "shape_msgs/MeshTriangle" :vertex_indices (make-array 3 :initial-contents '(0 5 4)))
                                       (roslisp:make-msg "shape_msgs/MeshTriangle" :vertex_indices (make-array 3 :initial-contents '(1 6 5)))
                                       (roslisp:make-msg "shape_msgs/MeshTriangle" :vertex_indices (make-array 3 :initial-contents '(2 7 6)))
                                       (roslisp:make-msg "shape_msgs/MeshTriangle" :vertex_indices (make-array 3 :initial-contents '(3 4 7)))))
           :vertices (make-array 8 :initial-contents
                                   (list
                                     (roslisp:make-msg "geometry_msgs/Point" :x -0.05 :y -0.05 :z -0.05)
                                     (roslisp:make-msg "geometry_msgs/Point" :x +0.05 :y -0.05 :z -0.05)
                                     (roslisp:make-msg "geometry_msgs/Point" :x +0.05 :y +0.05 :z -0.05)
                                     (roslisp:make-msg "geometry_msgs/Point" :x -0.05 :y +0.05 :z -0.05)
                                     (roslisp:make-msg "geometry_msgs/Point" :x -0.05 :y -0.05 :z +0.05)
                                     (roslisp:make-msg "geometry_msgs/Point" :x +0.05 :y -0.05 :z +0.05)
                                     (roslisp:make-msg "geometry_msgs/Point" :x +0.05 :y +0.05 :z +0.05)
                                     (roslisp:make-msg "geometry_msgs/Point" :x -0.05 :y +0.05 :z +0.05)))))
  (format T "Init done.~%"))

