;;; Copyright (c) 2012, Jan Winkler <winkler@cs.uni-bremen.de>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(in-package :pr2-manipulation-process-module)

(defparameter *pregrasp-offset-pose*
  (tf:make-pose
   (tf:make-3d-vector 0.20 0.0 0.0)
   (tf:euler->quaternion :az pi :ax (/ pi 2)))
  "Specifies the gripper pose relative to the respective handle
coordinate system (including it's origin and rotation) when going into
pregrasp.")
(defparameter *grasp-offset-pose*
  (tf:make-pose
   (tf:make-3d-vector 0.135 0.0 0.0)
   (tf:euler->quaternion :az pi :ax (/ pi 2)))
  "Specifies the gripper pose relative to the respective handle
coordinate system (including it's origin and rotation) when going into
grasp.")

;; These are for testing purposes at the moment. Handles must be
;; rotated, but the "normal" graspable objects are to be grasped using
;; their natural pose (plus a bit of a distance offset).
(defparameter *pregrasp-offset*
  (tf:make-pose
   (tf:make-3d-vector
    -0.25 0.0 0.0)
   (tf:euler->quaternion :ax (/ pi -2))))
(defparameter *grasp-offset*
  (tf:make-pose
   (tf:make-3d-vector
    -0.18 0.0 0.0)
   (tf:euler->quaternion :ax (/ pi -2))))
(defparameter *pre-putdown-offset*
  (tf:make-pose
   (tf:make-3d-vector
    0.0 0.0 0.2)
   (tf:euler->quaternion))) ;; Is there a rotation around X missing here?
(defparameter *putdown-offset*
  (tf:make-pose
   (tf:make-3d-vector
    0.0 0.0 0.0)
   (tf:euler->quaternion))) ;; Is there a rotation around X missing here?
(defparameter *unhand-offset*
  (tf:make-pose
   (tf:make-3d-vector
    -0.10 0.0 0.0)
   (tf:euler->quaternion))) ;; Is there a rotation around X missing here?

(defun relative-pose-for-handle (obj handle &key relative-pose)
  (tf:wait-for-transform *tf*
                         :timeout 5.0
                         :time (roslisp:ros-time)
                         :source-frame "map"
                         :target-frame "/map")
  (let* ((absolute-pose-map
           (absolute-handle
            obj handle
            :handle-offset-pose relative-pose))
         (absolute-pose (tf:transform-pose *tf*
                                           :pose absolute-pose-map
                                           :target-frame "/map")))
    absolute-pose))

(defun absolute-handle (obj handle
                        &key (handle-offset-pose
                              (tf:make-identity-pose)))
  "Transforms the relative handle location `handle' of object `obj'
into the object's coordinate system and returns the appropriate
location designator. The optional parameter `handle-offset-pose' is
applied to the handle pose before the absolute object pose is
applied."
  (let* ((absolute-object-loc (desig-prop-value obj 'at))
         (absolute-object-pose-stamped (pose-pointing-away-from-base
                                        (desig-prop-value
                                         absolute-object-loc
                                         'desig-props:pose)))
         (relative-handle-loc (desig-prop-value handle 'at))
         (relative-handle-pose (cl-transforms:transform-pose
                                (tf:pose->transform
                                 (reference relative-handle-loc))
                                handle-offset-pose))
         (pose-stamped (tf:pose->pose-stamped
                        (tf:frame-id absolute-object-pose-stamped)
                        (tf:stamp absolute-object-pose-stamped)
                        (cl-transforms:transform-pose
                         (tf:pose->transform
                          absolute-object-pose-stamped)
                         relative-handle-pose))))
    (make-designator 'object (loop for desc-elem in (description handle)
                                   when (eql (car desc-elem) 'at)
                                     collect `(at ,(make-designator
                                                    'location
                                                    `((pose ,pose-stamped))))
                                   when (not (eql (car desc-elem) 'at))
                                     collect desc-elem))))

(defun optimal-arm-handle-assignment (obj avail-arms avail-handles min-handles
                                      &key max-handles)
  (dolist (arm avail-arms)
    (open-gripper arm))
  (let* ((assigned-entities
           (entity-assignment
            (list
             (make-assignable-entity-list
              :entities avail-arms)
             (make-assignable-entity-list
              :entities avail-handles
              :min-assignments min-handles
              :max-assignments max-handles))))
         (valid-assignments
           (remove-if
            (lambda (assign)
              (not (cost-function-grasp-handle-ik-constraint-aware obj assign)))
            assigned-entities))
         (sorted-valid-assignments
           (sort
            valid-assignments
            (lambda (assign-1 assign-2)
              (let ((cost1 (cost-function-grasp-handle-ik-constraint-aware
                            obj assign-1))
                    (cost2 (cost-function-grasp-handle-ik-constraint-aware
                            obj assign-2)))
                (cond ((and cost1 cost2)
                       (< cost1 cost2))
                      (cost1 cost1)
                      (cost2 cost2))))))
         (assignments (mapcar (lambda (arm handle-pair)
                                (make-instance
                                 'grasp-assignment
                                 :pose (reference
                                        (desig-prop-value (cdr handle-pair) 'at))
                                 :side arm
                                 :handle-pair handle-pair))
                              (first (first sorted-valid-assignments))
                              (second (first sorted-valid-assignments)))))
    (unless assignments
      (cpl:fail 'cram-plan-failures:manipulation-pose-unreachable))
    assignments))

(defun cons-to-grasp-assignments (cons-cells)
  (mapcar (lambda (cons-cell)
            (make-instance 'grasp-assignment
                           :pose (cdr cons-cell)
                           :side (car cons-cell)))
          cons-cells))

(defun arms-handle-distances (arms handle-pose
                              &key
                                allowed-collision-objects
                                constraint-aware
                                (arms-offset-pose
                                 (tf:make-identity-pose)))
  "Calculates the distances for each arm given in the `arms' list with
respect to the handle `handle'. Only arms that can actually reach the
handle are included. The resulting list consists of entries of the
form `((\"with-offset\" . distance-with-offset) (\"without-offset\"
. distance-without-offset)' for each arm given in `arms'. If no arm
could reach the handle, `NIL' is returned."
  (flet ((apply-pose-offset (pose offset-pose)
           (cl-transforms:transform-pose
            (cl-transforms:pose->transform pose)
            offset-pose)))
    (loop for arm in arms
          for target-link = (ecase arm
                              (:left "l_wrist_roll_link")
                              (:right "r_wrist_roll_link"))
          for handle-pose-offsetted = (apply-pose-offset handle-pose
                                                         arms-offset-pose)
          for handle-pose-stamped = (tf:make-pose-stamped
                                     "/map"
                                     0.0
                                     (tf:origin handle-pose-offsetted)
                                     (tf:orientation handle-pose-offsetted))
          for distance = (reaching-length handle-pose-stamped arm
                                          :constraint-aware constraint-aware
                                          :calc-euclidean-distance nil
                                          :euclidean-target-link target-link
                                          :allowed-collision-objects
                                          allowed-collision-objects)
          when distance
            collect (cons arm distance))))

(defun optimal-arm-pose-assignment (obj avail-arms obj-pose)
  (dolist (arm avail-arms)
    (open-gripper arm))
  (let* ((object-name (desig-prop-value obj 'desig-props:name))
         (assigned-entities
           (entity-assignment
            (list
             (make-assignable-entity-list
              :entities avail-arms)
             (make-assignable-entity-list
              :entities (list obj-pose)))))
         (valid-assignments
           (remove-if
            (lambda (assign)
              (not (cost-function-grasp-ik-constraint-aware
                    obj assign
                    :allowed-collision-objects (list object-name))))
            assigned-entities))
         (sorted-valid-assignments
           (sort
            valid-assignments
            (lambda (assign-1 assign-2)
              (< (cost-function-grasp-ik-constraint-aware
                  obj assign-1
                  :allowed-collision-objects (list object-name))
                 (cost-function-grasp-ik-constraint-aware
                  obj assign-2
                  :allowed-collision-objects (list object-name))))))
         (assignments (mapcar (lambda (arm pose)
                                (make-instance 'grasp-assignment
                                               :pose pose
                                               :side arm))
                              (first (first sorted-valid-assignments))
                              (second (first sorted-valid-assignments)))))
    (unless assignments
      (cpl:fail 'cram-plan-failures:manipulation-pose-unreachable))
    assignments))

(defun cost-function-grasp-handle-ik-constraint-aware (obj assignment
                                                       &key
                                                         allowed-collision-objects)
  "This function determines the overall cost of the assignment
`assignment' with respect to the generated ik solutions (constraint
aware) and the cartesian distance between all points of this ik
solution. Physically speaking, this measures the distances the
individual arms have to travel when executing this grasp
configuration."
  (let ((assignment-poses
          (mapcar (lambda (handle-pair)
                    (reference (desig-prop-value (cdr handle-pair) 'at)))
                  (second assignment))))
    (cost-function-grasp-ik-constraint-aware
     obj (list (first assignment) assignment-poses)
     :allowed-collision-objects allowed-collision-objects)))

(defun cost-function-grasp-ik-constraint-aware (obj assignment
                                                &key
                                                  allowed-collision-objects)
  "This function determines the overall cost of the assignment
`assignment' with respect to the generated ik solutions (constraint
aware) and the cartesian distance between all points of this ik
solution. Physically speaking, this measures the distances the
individual arms have to travel when executing this grasp
configuration."
  ;; NOTE(winkler): The calculation of distances here is not flawless
  ;; actually. When determining the total distance, the pregrasp-pose
  ;; and the grasp-pose distance from the *current* pose is taken into
  ;; account. In reality, we only need the pregrasp->grasp distance in
  ;; the second step. This is a heuristic that works for now but could
  ;; be more sophisticated.
  (let ((costme (loop for (arm . pose) in (mapcar #'cons
                                    (first assignment)
                                    (second assignment))
        for distance-pregrasp = (cdr (assoc arm
                                            (arms-pose-distances
                                             (list arm) pose
                                             :arms-offset-pose
                                             *pregrasp-offset*)))
        for distance-grasp = (when distance-pregrasp
                               (moveit:remove-collision-object
                                (desig-prop-value obj 'desig-props:name))
                               (prog1
                                   (cdr (assoc arm
                                               (arms-pose-distances
                                                (list arm) pose
                                                :arms-offset-pose
                                                *grasp-offset*
                                                :allowed-collision-objects
                                                allowed-collision-objects)))
                                 (moveit:add-collision-object
                                  (desig-prop-value obj 'desig-props:name))))
        when (not (and distance-pregrasp distance-grasp))
          do (return-from cost-function-grasp-ik-constraint-aware nil)
        summing (+ distance-pregrasp distance-grasp) into total-cost
        finally (return total-cost))))
    costme))

(defun arms-pose-distances (arms pose
                            &key
                              allowed-collision-objects
                              (constraint-aware nil)
                              (arms-offset-pose
                               (tf:make-identity-pose)))
  (flet ((apply-pose-offset (pose offset-pose)
           (cl-transforms:transform-pose
            (cl-transforms:pose->transform pose)
            offset-pose)))
    (let ((costme
            (loop for arm in arms
                  for target-link = (ecase arm
                                      (:left "l_wrist_roll_link")
                                      (:right "r_wrist_roll_link"))
                  for pose-offsetted = (apply-pose-offset
                                        pose
                                        arms-offset-pose)
                  for pose-stamped = (tf:make-pose-stamped
                                      "/map"
                                      0.0
                                      (tf:origin pose-offsetted)
                                      (tf:orientation pose-offsetted))
                  for publ = (roslisp:publish
                              (roslisp:advertise
                               "/testpublisher2"
                               "geometry_msgs/PoseStamped")
                              (tf:pose-stamped->msg pose-stamped));;-tll))
                  for distance = (reaching-length
                                  pose-stamped arm
                                  :constraint-aware constraint-aware
                                  :calc-euclidean-distance t
                                  :euclidean-target-link target-link
                                  :allowed-collision-objects
                                  allowed-collision-objects)
                  when distance
                    collect (cons arm distance))))
      (roslisp:ros-info (pr2-manipulation-process-module grasping)
                        "IK solution cost: ~a~%" costme)
      costme)))

(defun pose-pointing-away-from-base (object-pose)
  (let ((ref-frame "/base_link")
        (fin-frame "/map"))
    (tf:wait-for-transform *tf* :source-frame ref-frame
                                :target-frame fin-frame
                                :time (roslisp:ros-time))
    (tf:wait-for-transform
     *tf*
     :source-frame (tf:frame-id object-pose)
     :target-frame fin-frame
     :time (roslisp:ros-time))
    (let* ((base-transform-map (tf:lookup-transform
                                *tf*
                                :time 0.0
                                :source-frame ref-frame
                                :target-frame fin-frame))
           (base-pose-map (tf:make-pose-stamped
                           (tf:frame-id base-transform-map)
                           (tf:stamp base-transform-map)
                           (tf:translation base-transform-map)
                           (tf:rotation base-transform-map)))
           (object-pose-map (tf:transform-pose
                             *tf*
                             :pose object-pose
                             :target-frame fin-frame))
           (origin1 (tf:origin base-pose-map))
           (origin2 (tf:origin object-pose-map))
           (p1 (tf:make-3d-vector (tf:x origin1) (tf:y origin1) 0.0))
           (p2 (tf:make-3d-vector (tf:x origin2) (tf:y origin2) 0.0))
           (angle (+ (* (signum (- (tf:y p2) (tf:y p1)))
                        (acos (/ (- (tf:x p2) (tf:x p1)) (tf:v-dist p1 p2))))
                     (/ pi -2))))
      (tf:make-pose-stamped fin-frame 0.0
                            (tf:origin object-pose-map)
                            (tf:euler->quaternion :az (+ angle (/ pi 2)))))))

(defun open-angle (object handle angle-deg)
  (let* ((angle-rad (* (/ angle-deg 180.0) pi))
         (object-pose (reference (desig-prop-value object 'desig-props:at)))
         (joint (desig-prop-value handle 'desig-props:joint))
         (joint-pose (reference (desig-prop-value joint 'desig-props:at)))
         (joint-axis (desig-prop-value joint 'desig-props:joint-axis))
         (handle-pose (reference (desig-prop-value handle 'desig-props:at))))
    (let* ((roll (* (tf:x joint-axis) angle-rad))
           (pitch (* (tf:y joint-axis) angle-rad))
           (yaw (* (tf:z joint-axis) angle-rad))
           (rot-q (tf:euler->quaternion :ax roll :ay pitch :az yaw)))
      (let* ((zeroed-handle-pose (tf:make-pose
                                  (tf:v- (tf:origin handle-pose)
                                         (tf:origin joint-pose))
                                  (tf:orientation handle-pose)))
             (rotated-handle-pose (cl-transforms:transform-pose
                                   (tf:make-transform
                                    (tf:make-identity-vector)
                                    rot-q)
                                   zeroed-handle-pose))
             (relative-handle-pose (cl-transforms:transform-pose
                                    (tf:make-transform
                                     (tf:origin joint-pose)
                                     (tf:make-identity-rotation))
                                    rotated-handle-pose))
             (absolute-handle-pose (cl-transforms:transform-pose
                                    (cl-transforms:pose->transform
                                     object-pose)
                                    relative-handle-pose)))
        (tf:pose->pose-stamped
         (tf:frame-id object-pose)
         (roslisp:ros-time)
         absolute-handle-pose)))))
