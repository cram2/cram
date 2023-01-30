;;;
;;; Copyright (c) 2018, Christopher Pollok <cpollok@uni-bremen.de>
;;;               2022, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(defparameter *handle-retract-offset* 0.05 "in meters")

(defmethod man-int:get-action-trajectory :heuristics 20 ((action-type (eql :opening))
                                                         arm
                                                         grasp
                                                         location
                                                         objects-acted-on
                                                         &key opening-distance)
  "Return a trajectory for opening the object in OBJECTS-ACTED-ON: mapPtool.
`opening-distance' describes how far the object should be opened in m."
  (declare (type keyword arm grasp)
           (type list objects-acted-on)
           (type number opening-distance))
  (when (not (eql 1 (length objects-acted-on)))
    (error "Action-type ~a requires exactly one object.~%" action-type))
  (make-trajectory action-type arm objects-acted-on opening-distance grasp))

(defmethod man-int:get-action-trajectory :heuristics 20 ((action-type (eql :closing))
                                                         arm
                                                         grasp
                                                         location
                                                         objects-acted-on
                                                         &key opening-distance)
  "Return a trajectory for closing the object in OBJECTS-ACTED-ON: mapPtool.
`opening-distance' describes how far the object should be closed in m. "
  (declare (type keyword arm grasp)
           (type list objects-acted-on)
           (type number opening-distance))
  (when (not (eql 1 (length objects-acted-on)))
    (error "Action-type ~a requires exactly one object.~%" action-type))
  (make-trajectory action-type arm objects-acted-on opening-distance grasp))

(defun make-trajectory (action-type arm objects-acted-on opening-distance grasp)
  "Make a trajectory for opening or closing a container. Returns a list of mapPtool.
This should only be called by `get-action-trajectory' for action-types :opening and :closing.
The parameters are analog to the ones of `get-action-trajectory'."
  (declare (type keyword action-type arm grasp)
           (type list objects-acted-on)
           (type number opening-distance))
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
             (get-environment-link object-name object-environment))))
         (base-T-object-name-stamped
           (second
            (get-container-pose-and-transform
             manipulated-link-name object-environment)))
         (object-name-T-tool-grasp-stamped
           (get-container-to-gripper-transform
            manipulated-link-name arm grasp object-environment :grasp))
         (object-name-T-tool-pregrasp-stamped
           (get-container-to-gripper-transform
            manipulated-link-name arm grasp object-environment :pregrasp))
         (joint-axis
           (get-container-axis object-name object-environment)))

    ;; checks if `object-type' is a subtype of :container-prismatic or :container-revolute
    ;; and executes the corresponding MAKE-PRISMATIC-TRAJECTORY or MAKE-REVOLUTE-TRAJECTORY.
    (alexandria:switch
        (object-type :test (lambda (?type ?super-type)
                             (prolog:prolog
                              `(man-int:object-type-subtype ,?super-type ,?type))))
      (:container-prismatic
       (make-prismatic-trajectory base-T-object-name-stamped arm action-type
                                  object-name-T-tool-grasp-stamped
                                  object-name-T-tool-pregrasp-stamped
                                  opening-distance joint-axis))
      (:container-revolute
       (make-revolute-trajectory base-T-object-name-stamped arm action-type
                                 object-name-T-tool-grasp-stamped
                                 object-name-T-tool-pregrasp-stamped
                                 opening-distance joint-axis))
      (T (error "Unsupported container-type: ~a." object-type)))))


(defun make-prismatic-trajectory (base-T-object-name-stamped arm action-type
                                  object-name-T-tool-grasp-stamped
                                  object-name-T-tool-pregrasp-stamped
                                  opening-distance axis)
  "Return a list of `man-int::traj-segment's representing a trajectory to open a
container with prismatic joints.
`base-T-object-name-stamped' should have `cram-tf:*robot-base-frame*'
as it's frame and the object's frame as the child.
`arm' is the arm that should be used (eg. :left or :right)
`action-type' is :opening or :closing
`object-name-T-tool-grasp-stamped' is a transform with the object's frame and the
frame of the robot's end effector as the child (eg. `cram-tf:*robot-left-tool-frame*).
`opening-distance' is the distance the object should be manipulated in m."
  (declare (type cl-transforms-stamped:transform-stamped
                 base-T-object-name-stamped object-name-T-tool-grasp-stamped
                 object-name-T-tool-pregrasp-stamped)
           (type keyword arm action-type)
           (type number opening-distance))

  (let* ((traj-poses (get-prismatic-traj-poses object-name-T-tool-grasp-stamped
                                               :opening-distance opening-distance
                                               :axis axis))
         (last-traj-pose (car (last traj-poses))))
    (mapcar
     (lambda (label transforms)
       (man-int:make-traj-segment
        :label label
        :poses (mapcar (alexandria:curry #'man-int:calculate-gripper-pose-in-map
                                         base-T-object-name-stamped arm)
                       transforms)))
     `(:reaching
       :grasping
       ,action-type
       :retracting)
     (list
      (list object-name-T-tool-pregrasp-stamped)
      (list object-name-T-tool-grasp-stamped)
      traj-poses
      (list (cram-tf:apply-transform
             last-traj-pose
             (cl-transforms-stamped:make-transform-stamped
              (cl-transforms-stamped:child-frame-id last-traj-pose)
              (cl-transforms-stamped:child-frame-id last-traj-pose)
              (cl-transforms-stamped:stamp last-traj-pose)
              (cl-transforms:make-3d-vector
               0 0 (- *handle-retract-offset*))
              (cl-transforms:make-identity-rotation))))))))

(defun get-prismatic-traj-poses (object-name-T-tool-grasp-stamped
                                 &key axis opening-distance)
  (list
   (cram-tf:multiply-transform-stampeds
    (cl-transforms-stamped:frame-id object-name-T-tool-grasp-stamped)
    (cl-transforms-stamped:child-frame-id object-name-T-tool-grasp-stamped)
    (cl-transforms-stamped:make-transform-stamped
     (cl-transforms-stamped:frame-id object-name-T-tool-grasp-stamped)
     (cl-transforms-stamped:frame-id object-name-T-tool-grasp-stamped)
     0.0
     (cl-transforms:v* axis opening-distance)
     (cl-transforms:make-identity-rotation))
    object-name-T-tool-grasp-stamped)))

(defun make-revolute-trajectory (base-T-object-name-stamped arm action-type
                                 object-name-T-tool-grasp-stamped
                                 object-name-T-tool-pregrasp-stamped
                                 opening-angle
                                 axis)
  "Return a list of `man-int::traj-segment' representing a trajectory to open a
container with revolute joints.
`base-T-object-name-stamped' should have `cram-tf:*robot-base-frame*'
as it's frame and the object's frame as the child.
`arm' is the arm that should be used (eg. :left or :right)
`action-type' is :opening or :closing
`object-name-T-tool-grasp-stamped' is a transform with the object's frame and the
frame of the robot's end effector as the child (eg. `cram-tf:*robot-left-tool-frame*).
`opening-distance' is the distance the object should be manipulated in rad."
  (declare (type cl-transforms-stamped:transform-stamped
                 base-T-object-name-stamped object-name-T-tool-grasp-stamped
                 object-name-T-tool-pregrasp-stamped)
           (type keyword arm action-type)
           (type number opening-angle))
  (let* ((traj-poses (get-revolute-traj-poses object-name-T-tool-grasp-stamped
                                              :angle-max opening-angle :axis axis))
         (last-traj-pose (car (last traj-poses))))
    (mapcar
     (lambda (label transforms)
       (man-int:make-traj-segment
        :label label
        :poses (mapcar (alexandria:curry #'man-int:calculate-gripper-pose-in-map
                                         base-T-object-name-stamped arm)
                       transforms)))
     `(:reaching
       :grasping
       ,action-type
       :retracting)
     (list
      (list object-name-T-tool-pregrasp-stamped)
      (list object-name-T-tool-grasp-stamped)
      traj-poses
      (list (cram-tf:apply-transform
             last-traj-pose
             (cl-transforms-stamped:make-transform-stamped
              (cl-transforms-stamped:child-frame-id last-traj-pose)
              (cl-transforms-stamped:child-frame-id last-traj-pose)
              (cl-transforms-stamped:stamp last-traj-pose)
              (cl-transforms:make-3d-vector
               0 0 (- *handle-retract-offset*))
              (cl-transforms:make-identity-rotation))))))))


(defun 3d-vector->keyparam-list (v)
  "Convert a vector into a list with content
   (:AX <x-value> :AY <y-value> :AZ <z-value>)."
  (declare (type cl-transforms:3d-vector v))
  (list
   :ax (cl-transforms:x v)
   :ay (cl-transforms:y v)
   :az (cl-transforms:z v)))

(defun get-revolute-traj-poses (container-T-tool
                                &key
                                  (axis (cl-transforms:make-3d-vector 0 0 1))
                                  angle-max)
  "Return a list of stamped transforms from of the gripper-frame in the joint-frame rotated
around `axis' by `angle-max' in steps of 0.1 rad."
  (declare (type cl-transforms-stamped:transform-stamped container-T-tool)
           (type cl-transforms:3d-vector axis)
           (type number angle-max))
  (let* ((angle-step (if (>= angle-max 0)
                         0.1
                         -0.1))
         (angles (append (loop for angle = 0.0 then (+ angle angle-step)
                               while (< (abs angle) (abs angle-max))
                               collect angle)
                         (list angle-max))))
    (mapcar (lambda (angle)
              (let ((rotation (cl-transforms:axis-angle->quaternion axis angle)))
                (cl-transforms-stamped:make-transform-stamped
                 (cl-transforms-stamped:frame-id container-T-tool)
                 (cl-transforms-stamped:child-frame-id container-T-tool)
                 (cl-transforms-stamped:stamp container-T-tool)
                 (cl-transforms:rotate rotation
                                       (cl-transforms:translation container-T-tool))
                 (cl-transforms:q* rotation
                                   (cl-transforms:rotation container-T-tool)))))
            angles)))

(defun get-container-to-gripper-transform (object-name
                                           arm
                                           grasp-pose-name
                                           btr-environment
                                           pregrasp-or-grasp)
  "Get the transform of the robot's gripper in the container frame, i.e.,
object-name-T-tool, where tool is for the standard gripper frame.
`object-name' is the name of a container in the `btr-environment'.
`arm' denotes which arm's gripper should be used (eg. :left or :right).
`pregrasp-or-grasp' can be either :pregrasp or :grasp.
If :pregrasp, object-name-T-tool-pregrasp is returned, otherwise object-name-T-tool-grasp.
`btr-environment' is the name of the environment in which the container is located (eg. :KITCHEN)."
  (declare (type (or string symbol) object-name)
           (type keyword arm grasp-pose-name btr-environment pregrasp-or-grasp))
  (when (symbolp object-name)
    (setf object-name
          (roslisp-utilities:rosify-underscores-lisp-name object-name)))
  (let* ((handle-name
           (cl-urdf:name (get-handle-link object-name btr-environment)))
         (handle-name-keyword
           (roslisp-utilities:lispify-ros-underscore-name handle-name :keyword))
         (map-T-handle-stamped
           (cram-tf:pose->transform-stamped
            cram-tf:*fixed-frame*
            handle-name
            0
            (get-urdf-link-pose handle-name btr-environment)))
         (map-T-object-name-stamped
           (cram-tf:pose->transform-stamped
            cram-tf:*fixed-frame*
            object-name
            0
            (get-urdf-link-pose object-name btr-environment)))
         (tool-frame
           (ecase arm
             (:left cram-tf:*robot-left-tool-frame*)
             (:right cram-tf:*robot-right-tool-frame*)))
         (object-name-T-map-stamped
           (cram-tf:transform-stamped-inv map-T-object-name-stamped))
         (object-name-T-handle-stamped
           (cram-tf:multiply-transform-stampeds
            object-name handle-name
            object-name-T-map-stamped map-T-handle-stamped))
         (handle-T-tool-grasp-stamped
           (man-int:get-object-type-to-gripper-transform
            :handle handle-name-keyword arm grasp-pose-name))
         (handle-T-tool-stamped
           (if (eq pregrasp-or-grasp :pregrasp)
               (car (man-int:get-object-type-to-gripper-pregrasp-transforms
                     :handle handle-name-keyword arm grasp-pose-name nil
                     handle-T-tool-grasp-stamped))
               handle-T-tool-grasp-stamped))
         (object-name-T-tool-stamped
           (cram-tf:multiply-transform-stampeds
            object-name tool-frame
            object-name-T-handle-stamped handle-T-tool-stamped)))
    object-name-T-tool-stamped))

(defun get-container-axis (object-name object-environment)
  (let* ((door-link (get-environment-link object-name object-environment))
         (door-joint (get-connecting-joint door-link))
         (door-joint-axis (cl-urdf:axis door-joint)))
    door-joint-axis))
