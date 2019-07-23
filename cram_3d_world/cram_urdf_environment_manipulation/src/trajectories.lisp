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

(in-package :env-man)

(defparameter *drawer-handle-grasp-x-offset* 0.0 "in meters")
(defparameter *drawer-handle-pregrasp-x-offset* 0.10 "in meters")
(defparameter *drawer-handle-retract-offset* 0.10 "in meters")
(defparameter *door-handle-retract-offset* 0.05 "in meters")

(defmethod man-int:get-action-trajectory :heuristics 20 ((action-type (eql :opening))
                                                         arm
                                                         grasp
                                                         objects-acted-on
                                                         &key
                                                           opening-distance
                                                           handle-axis)
  "Return a trajectory for opening the object in OBJECTS-ACTED-ON.
OPENING-DISTANCE is a float in m, describing how far the object should opened.
HANDLE-AXIS is a `cl-transforms:3d-vector' describing the handle's orientation
in the robot's XZ-plane. It's Z-element should be 0."
  (when (not (eql 1 (length objects-acted-on)))
    (error "Action-type ~a requires exactly one object.~%" action-type))
  (make-trajectory action-type arm objects-acted-on opening-distance handle-axis))

(defmethod man-int:get-action-trajectory :heuristics 20 ((action-type (eql :closing))
                                                         arm
                                                         grasp
                                                         objects-acted-on
                                                         &key
                                                           opening-distance
                                                           handle-axis)
  "Return a trajectory for closing the object in OBJECTS-ACTED-ON.
OPENING-DISTANCE is a float in m, describing how far the object should closed.
HANDLE-AXIS is a `cl-transforms:3d-vector' describing the handle's orientation
in the robot's XZ-plane. It's Z-element should be 0."
  (when (not (eql 1 (length objects-acted-on)))
    (error "Action-type ~a requires exactly one object.~%" action-type))
  (make-trajectory action-type arm objects-acted-on opening-distance handle-axis))

(defun make-trajectory (action-type
                        arm
                        objects-acted-on
                        opening-distance
                        handle-axis)
  "Make a trajectory for opening or closing a container.
This should only be used by get-action-trajectory for action-types :opening and :closing."
  (when (equal action-type :closing)
    (setf opening-distance (- opening-distance)))
  (let* ((object-designator
           (car objects-acted-on))
         (object-name
           (desig:desig-prop-value object-designator :urdf-name))
         (object-type
           (desig:desig-prop-value object-designator :type))
         (object-environment
           (desig:desig-prop-value object-designator :part-of))
         (manipulated-link-name
           (cl-urdf:name
            (get-manipulated-link
             (get-container-link object-name object-environment))))
         (object-transform
           (second
            (get-container-pose-and-transform manipulated-link-name object-environment)))
         (grasp-pose
           (get-container-to-gripper-transform manipulated-link-name arm handle-axis object-environment)))

    ;; checks if `object-type' is a subtype of :container-prismatic or :container-revolute
    ;; and executes the corresponding MAKE-PRISMATIC-TRAJECTORY or MAKE-REVOLUTE-TRAJECTORY.
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
  "Return a list of `man-int::traj-segment' representing a trajectory to open a
container with prismatic joints."
  (mapcar
   (lambda (label transform)
     (man-int:make-traj-segment
      :label label
      :poses (list (man-int:calculate-gripper-pose-in-map object-transform arm transform))))
   `(:reaching
     :grasping
     ,action-type
     :retracting)
   (list
    (cram-tf:translate-transform-stamped
     grasp-pose :x-offset *drawer-handle-pregrasp-x-offset*)
    grasp-pose
    (cram-tf:translate-transform-stamped
     grasp-pose :x-offset opening-distance)
    (cram-tf:translate-transform-stamped
     grasp-pose :x-offset (+ opening-distance *drawer-handle-retract-offset*)))))

(defun make-revolute-trajectory (object-transform arm action-type
                                 grasp-pose opening-angle)
  "Return a list of `man-int::traj-segment' representing a trajectory to open a
container with revolute joints."
  (let* ((traj-poses (get-revolute-traj-poses grasp-pose :angle-max opening-angle))
         (last-traj-pose (car (last traj-poses))))
    (mapcar
     (lambda (label transforms)
       (man-int:make-traj-segment
        :label label
        :poses (mapcar (alexandria:curry #'man-int:calculate-gripper-pose-in-map
                                         object-transform arm)
                       transforms)))
     `(:reaching
       :grasping
       ,action-type
       :retracting)
     (list
      (list (cram-tf:translate-transform-stamped
             grasp-pose :x-offset *drawer-handle-pregrasp-x-offset*))
      (list grasp-pose)
      traj-poses
      (when last-traj-pose
        (list (cram-tf:apply-transform
               last-traj-pose
               (cl-transforms-stamped:make-transform-stamped
                (cl-transforms-stamped:child-frame-id last-traj-pose)
                (cl-transforms-stamped:child-frame-id last-traj-pose)
                (cl-transforms-stamped:stamp last-traj-pose)
                (cl-transforms:make-3d-vector 0 0 -0.1)
                (cl-transforms:make-identity-rotation)))))))))


(defun 3d-vector->keyparam-list (v)
  "Convert a `cl-transform:3d-vector' into a list with content
   (:AX <x-value> :AY <y-value> :AZ <z-value>)."
  (declare (type cl-transforms:3d-vector v))
  (list
   :ax (cl-transforms:x v)
   :ay (cl-transforms:y v)
   :az (cl-transforms:z v)))

(defun get-revolute-traj-poses (joint-to-gripper
                                &key
                                  (axis (cl-transforms:make-3d-vector 0 0 1))
                                  (angle-max (cram-math:degrees->radians 80)))
  "Return a list of transforms from joint to gripper rotated around AXIS by ANGLE-MAX."
  (let ((angle-step (if (>= angle-max 0)
                        0.1
                        -0.1)))
    (loop for angle = 0.0 then (+ angle angle-step)
          while (< (abs angle) (abs angle-max))
          collect
          (let ((rotation
                  (cl-transforms:axis-angle->quaternion axis angle)))
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


(defun get-container-to-gripper-transform (object-name
                                           arm
                                           handle-axis
                                           btr-environment)
  "Get the transform from the container handle to the robot's gripper."
  (when (symbolp object-name)
    (setf object-name
          (roslisp-utilities:rosify-underscores-lisp-name object-name)))
  (let* ((handle-name
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
      ;; Calculate the grippers orientation, so it's properly aligned with the
      ;; handle.
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
        0))))))
