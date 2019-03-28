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
(defparameter *door-handle-retract-offset* 0.05 "in meters")

(defmethod man-int:get-object-type-gripper-opening
    ((object-type (eql :container))) 0.10)
(defmethod man-int:get-object-type-gripper-opening
    ((object-type (eql :container-prismatic))) 0.10)
(defmethod man-int:get-object-type-gripper-opening
    ((object-type (eql :container-revolute))) 0.10)

;; TODO(cpo): Make handle-angle be an actual angle of some kind, so we can
;; handle more than horizontal and vertical
(defun get-container-to-gripper-transform (object-name
                                           arm
                                           handle-axis
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
             (:right cram-tf:*robot-right-tool-frame*)))
         (handle-angle-cos
           (angle-between-vectors
            (cl-transforms:make-3d-vector 1 0 0)
            handle-axis))
         (handle-angle-sin (- 1 handle-angle-cos)))
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
      (cl-transforms:column-vectors->quaternion
       (cl-transforms:make-3d-vector
        0
        (- handle-angle-sin)
        handle-angle-cos)
       (cl-transforms:make-3d-vector
        0
        handle-angle-cos
        handle-angle-sin)
       (cl-transforms:make-3d-vector
        -1
        0
        0))
      ;;(cl-transforms:matrix->quaternion
       ;; (if (eq handle-axis :horizontal)
       ;;     #2A((0 0 -1)
       ;;         (0 1 0)
       ;;         (1 0 0))
       ;;     #2A((0 0 -1)
       ;;         (-1 0 0)
       ;;         (0 1 0)))
       ;; #2A((0 0 -1)
       ;;     ((- (- 1 handle-angle-cos)) handle-angle-cos 0)
       ;;     (handle-angle-cos (- 1 handle-angle-cos) 0))
      ;; )
      ))))

(defmethod man-int:get-action-trajectory :before ((action-type (eql :opening))
                                                   arm
                                                   grasp
                                                   objects-acted-on
                                                   &key
                                                     opening-distance
                                                     handle-axis)
  "Raise an error if object count is not right."
  (declare (ignore arm grasp opening-distance handle-axis))
  (when (not (eql 1 (length objects-acted-on)))
    (error (format nil "Action-type ~a requires exactly one object.~%" action-type))))

(defmethod man-int:get-action-trajectory :before ((action-type (eql :closing))
                                                   arm
                                                   grasp
                                                   objects-acted-on
                                                   &key
                                                     opening-distance
                                                     handle-axis)
  "Raise an error if object count is not right."
  (declare (ignore arm grasp opening-distance handle-axis))
  (when (not (eql 1 (length objects-acted-on)))
    (error (format nil "Action-type ~a requires exactly one object.~%" action-type))))

(defmethod man-int:get-action-trajectory ((action-type (eql :opening))
                                           arm
                                           grasp
                                           objects-acted-on
                                           &key
                                             opening-distance
                                             handle-axis)
  (make-trajectory action-type arm objects-acted-on opening-distance handle-axis))

(defmethod man-int:get-action-trajectory ((action-type (eql :closing))
                                           arm
                                           grasp
                                           objects-acted-on
                                           &key
                                             opening-distance
                                             handle-axis)
  (make-trajectory action-type arm objects-acted-on opening-distance handle-axis))

(defun make-trajectory (action-type
                        arm
                        objects-acted-on
                        opening-distance
                        handle-axis)
  "Make a trajectory for opening or closing a container.
   This should only be used by get-action-trajectory for action-types :opening and
   :closing."
  (when (equal action-type :closing)
    (setf opening-distance (- opening-distance)))
  (let* ((object-designator (car objects-acted-on))
         (object-name
           (desig:desig-prop-value
            object-designator :urdf-name))
         (object-type
           (desig:desig-prop-value
            object-designator :type))
         (object-environment
           (desig:desig-prop-value
            object-designator :part-of))
         (object-transform
           (second
            (get-container-pose-and-transform
             object-name
             object-environment)))
         (grasp-pose
           (get-container-to-gripper-transform
            object-name
            arm
            handle-axis
            object-environment)))

    (alexandria:switch
        (object-type :test (lambda (?type ?super-type)
                             (prolog:prolog
                              `(man-int:object-type-subtype
                                ,?super-type ,?type))))
      (:container-prismatic
       (make-prismatic-trajectory object-transform arm action-type grasp-pose opening-distance))
      (:container-revolute
       (make-revolute-trajectory object-transform arm action-type grasp-pose opening-distance))
      (T (error "Unsupported container-type: ~a." object-type)))))

(defun make-prismatic-trajectory (object-transform arm action-type
                                  grasp-pose opening-distance)
  (mapcar (lambda (label transforms)
                (man-int:make-traj-segment
                 :label label
                 :poses (mapcar (man-int:make-object-to-standard-gripper->base-to-particular-gripper-transformer
                                 object-transform arm)
                                transforms)))
              (list
               :reaching
               :grasping
               action-type
               :retracting)
              (list
               (list (cram-tf:translate-transform-stamped
                      grasp-pose :x-offset *drawer-handle-pregrasp-x-offset*))
               (list grasp-pose)
               (list (cram-tf:translate-transform-stamped
                      grasp-pose :x-offset opening-distance))
               (list (cram-tf:translate-transform-stamped
                      grasp-pose :x-offset (+ opening-distance *drawer-handle-retract-offset*))))))

(defun make-revolute-trajectory (object-transform arm action-type
                                 grasp-pose opening-angle)
  (let* ((traj-poses (get-revolute-traj-poses grasp-pose :angle-max opening-angle)))
    (mapcar (lambda (label transforms)
              (man-int:make-traj-segment
               :label label
               :poses
               (mapcar
                (man-int:make-object-to-standard-gripper->base-to-particular-gripper-transformer
                               object-transform arm)
                              transforms)))
            (list
             :reaching
             :grasping
             action-type
             :retracting)
            (list
             (list (cram-tf:translate-transform-stamped
                    grasp-pose :x-offset *drawer-handle-pregrasp-x-offset*))
             (list grasp-pose)
             traj-poses
             (let ((last-traj-pose (car (last traj-poses))))
               (list (cram-tf:apply-transform
                      last-traj-pose
                      (cl-transforms-stamped:make-transform-stamped
                       (cl-transforms-stamped:child-frame-id last-traj-pose)
                       (cl-transforms-stamped:child-frame-id last-traj-pose)
                       (cl-transforms-stamped:stamp last-traj-pose)
                       (cl-transforms:make-3d-vector 0 0 -0.1)
                       (cl-transforms:make-identity-rotation)))))))))

;;TODO(cpo): Move to cram-tf or -math?
(defun 3d-vector->keyparam-list (v)
  "Convert a cl-transform:3d-vector into a list with content
   (:AX <x-value> :AY <y-value> :AZ <z-value>)."
  (declare (type cl-transforms:3d-vector v))
  (list
   :ax (cl-transforms:x v)
   :ay (cl-transforms:y v)
   :az (cl-transforms:z v)))

(defun get-revolute-traj-poses
    (joint-to-gripper
     &key
       (axis (cl-transforms:make-3d-vector 0 0 1))
       (angle-max (cram-math:degrees->radians 80)))
  (let ((angle-step (if (>= angle-max 0)
                        0.1
                        -0.1)))
    (loop for angle = 0.0 then (+ angle angle-step)
          while (< (abs angle) (abs angle-max))
          collect
          (let ((rotation
                  (cl-transforms:axis-angle->quaternion
                   axis angle)))
            (cl-transforms-stamped:make-transform-stamped
             (cl-transforms-stamped:frame-id joint-to-gripper)
             (cl-transforms-stamped:child-frame-id joint-to-gripper)
             (cl-transforms-stamped:stamp joint-to-gripper)
             (cl-transforms:rotate rotation (cl-tf:translation joint-to-gripper))
             (apply 'cl-transforms:euler->quaternion
                    (3d-vector->keyparam-list
                     (cl-transforms:v+
                      (apply 'cl-transforms:make-3d-vector
                             (cl-transforms:quaternion->euler
                              (cl-transforms:rotation joint-to-gripper) :just-values t))
                      (cl-transforms:v*
                       axis
                       angle)))))))))
