;;;
;;; Copyright (c) 2018, Christopher Pollok <cpollok@uni-bremen.de>
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

(in-package :pr2-em)

(defparameter *drawer-handle-grasp-x-offset* 0.0 "in meters")
(defparameter *drawer-handle-pregrasp-x-offset* 0.10 "in meters")
(defparameter *drawer-handle-retract-offset* 0.10 "in meters")

(defmethod man-int:get-object-type-gripper-opening ((object-type (eql :container))) 0.10)

(defun get-container-to-gripper-transform (object-name
                                           arm
                                           btr-environment)
  "Get the transform from the container handle to the robot's gripper."
  (let* ((object-name
           (roslisp-utilities:rosify-underscores-lisp-name object-name))
         (handle-name
           (cl-urdf:name (get-handle-link object-name btr-environment)))
         (handle-tf
           (cl-transforms-stamped:transform->transform-stamped
            cram-tf:*fixed-frame*
            handle-name
            0
            (cl-transforms:pose->transform
             (get-urdf-link-pose handle-name btr-environment))))
         (container-tf
           (cl-transforms-stamped:transform->transform-stamped
            cram-tf:*fixed-frame*
            object-name
            0
            (cl-transforms:pose->transform
             (get-urdf-link-pose object-name btr-environment))))
         (tool-frame
           (ecase arm
             (:left cram-tf:*robot-left-tool-frame*)
             (:right cram-tf:*robot-right-tool-frame*))))
    (cram-tf:multiply-transform-stampeds
     object-name
     tool-frame
     (cram-tf:multiply-transform-stampeds
      object-name
      handle-name
      (cram-tf:transform-stamped-inv container-tf)
      handle-tf)
     (cl-transforms-stamped:make-transform-stamped
      handle-name
      tool-frame
      0.0
      (cl-transforms:make-3d-vector *drawer-handle-grasp-x-offset* 0.0d0 0.0d0)
      (cl-transforms:matrix->quaternion
       #2A((0 0 -1)
           (0 1 0)
           (1 0 0)))))))

(defmethod man-int:get-action-trajectory :before ((action-type (eql :opening))
                                                   arm
                                                   grasp
                                                   objects-acted-on
                                                   &key
                                                     opening-distance)
  "Raise an error if object count is not right."
  (declare (ignore arm grasp opening-distance))
  (when (not (eql 1 (length objects-acted-on)))
    (error (format nil "Action-type ~a requires exactly one object.~%" action-type))))

(defmethod man-int:get-action-trajectory :before ((action-type (eql :closing))
                                                   arm
                                                   grasp
                                                   objects-acted-on
                                                   &key
                                                     opening-distance)
  "Raise an error if object count is not right."
  (declare (ignore arm grasp opening-distance))
  (when (not (eql 1 (length objects-acted-on)))
    (error (format nil "Action-type ~a requires exactly one object.~%" action-type))))

(defun make-trajectory (action-type
                        arm
                        objects-acted-on
                        opening-distance)
  "Make a trajectory for opening or closing a container.
   This should only be used by get-action-trajectory for action-types :opening and
   :closing."
  (when (equal action-type :closing)
    (setf opening-distance (- opening-distance)))
  (let* ((object-designator (car objects-acted-on))
         (object-name
           (desig:desig-prop-value
            object-designator :urdf-name))
         (object-environment
           (desig:desig-prop-value
            object-designator :part-of))
         (object-transform
           (second
            (get-container-pose-and-transform
             object-name
             object-environment)))
         (gripper-tool-frame
           (ecase arm
             (:left cram-tf:*robot-left-tool-frame*)
             (:right cram-tf:*robot-right-tool-frame*)))
         (grasp-pose
           (get-container-to-gripper-transform
            object-name
            arm
            object-environment))
         (standard-to-particular-gripper-transform ; g'Tg
           (cl-transforms-stamped:transform->transform-stamped
            gripper-tool-frame
            gripper-tool-frame
            0.0
            (cut:var-value
             '?transform
             (car (prolog:prolog
                   `(and (cram-robot-interfaces:robot ?robot)
                         (cram-robot-interfaces:standard-to-particular-gripper-transform
                          ?robot ?transform))))))))
    
    (let ((object-to-standard-gripper->base-to-particular-gripper
            (man-int:make-object-to-standard-gripper->base-to-particular-gripper-transformer
             object-transform gripper-tool-frame standard-to-particular-gripper-transform)))
      (mapcar (lambda (label transform)
                (man-int::make-traj-segment
                 :label label
                 :poses (list
                         (funcall object-to-standard-gripper->base-to-particular-gripper
                                  transform))))
                (list
                 :reach
                 :grasp
                 :close
                 :retract)
                (list
                 (cram-tf:translate-transform-stamped
                  grasp-pose :x-offset *drawer-handle-pregrasp-x-offset*)
                 grasp-pose
                 (cram-tf:translate-transform-stamped
                  grasp-pose :x-offset opening-distance)
                 (cram-tf:translate-transform-stamped
                  grasp-pose :x-offset (+ opening-distance *drawer-handle-retract-offset*)))))))

(defmethod man-int:get-action-trajectory ((action-type (eql :opening))
                                           arm
                                           grasp
                                           objects-acted-on
                                           &key
                                             opening-distance)
  (make-trajectory action-type arm objects-acted-on opening-distance))

(defmethod man-int:get-action-trajectory ((action-type (eql :closing))
                                           arm
                                           grasp
                                           objects-acted-on
                                           &key
                                             opening-distance)
  (make-trajectory action-type arm objects-acted-on opening-distance))
