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

(defparameter *pregrasp-offset*
  (tf:make-pose
   (tf:make-3d-vector
    -0.31 0.0 0.0)
   (tf:euler->quaternion :ax (/ pi -2))))
(defparameter *grasp-offset*
  (tf:make-pose
   (tf:make-3d-vector
    -0.19 0.0 0.0)
   (tf:euler->quaternion :ax (/ pi -2))))
(defparameter *pre-putdown-offset*
  (tf:make-pose
   (tf:make-3d-vector
    0.0 0.0 0.2)
   (tf:euler->quaternion)))
(defparameter *putdown-offset*
  (tf:make-pose
   (tf:make-3d-vector
    0.0 0.0 0.0)
   (tf:euler->quaternion)))
(defparameter *unhand-offset*
  (tf:make-pose
   (tf:make-3d-vector
    -0.10 0.0 0.0)
   (tf:euler->quaternion)))

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
                                      &key (max-handles
                                            (or (desig-prop-value
                                                 obj 'desig-props:max-handles)
                                                nil)))
  (declare (ignore max-handles))
  (assert (= min-handles 1)
          () "Sorry, not more than one handle at a time right now.")
  (ros-info (pr2 manip-pm) "Opening grippers")
  (dolist (arm avail-arms)
    (open-gripper arm))
  (ros-info (pr2 manip-pm) "Calculating optimal grasp: ~a arms, ~a handles (min ~a)" (length avail-arms) (length avail-handles) min-handles)
  (let* ((assignments
           (alexandria:flatten
            (loop for i from 0 below (length avail-arms)
                  for arm = (nth i avail-arms)
                  for sublist = (loop for j from 0 below (length avail-handles)
                                      for handle = (nth i avail-handles)
                                      for cost = (cost-function-ik-handle
                                                  obj (list (list (nth i avail-arms))
                                                            (list (nth j avail-handles))))
                                      when cost
                                        collect (make-instance
                                                 'grasp-assignment
                                                 :handle-pair handle
                                                 :side arm
                                                 :ik-cost cost
                                                 :pose (reference
                                                        (desig-prop-value
                                                         (cdr handle) 'at))))
                  when sublist
                    collect sublist)))
         (sorted-assignments (sort assignments #'< :key #'ik-cost)))
    (ros-info (pr2 manip-pm) "Done calculating. Got ~a/~a result(s)."
              (length assignments) (length sorted-assignments))
    (unless sorted-assignments
      (cpl:fail 'cram-plan-failures:manipulation-pose-unreachable))
    (list (first sorted-assignments))))

(defun cons-to-grasp-assignments (cons-cells)
  (mapcar (lambda (cons-cell)
            (make-instance 'grasp-assignment
                           :pose (cdr cons-cell)
                           :side (car cons-cell)))
          cons-cells))

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
              (not (cost-function-ik-pose
                    obj assign
                    :allowed-collision-objects (list object-name))))
            assigned-entities))
         (sorted-valid-assignments
           (sort
            valid-assignments
            (lambda (assign-1 assign-2)
              (< (cost-function-ik-pose
                  obj assign-1
                  :allowed-collision-objects (list object-name))
                 (cost-function-ik-pose
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

(defun cost-function-ik-handle (obj assignment &key allowed-collision-objects)
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
    (cost-function-ik-pose
     obj (list (first assignment) assignment-poses)
     :allowed-collision-objects allowed-collision-objects)))

(defun cost-function-ik-pose (obj assignment &key allowed-collision-objects)
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
                                             *pregrasp-offset*
                                             :highlight-links
                                             (links-for-arm-side arm))))
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
                                                allowed-collision-objects
                                                :highlight-links
                                                (links-for-arm-side arm))))
                                 (moveit:add-collision-object
                                  (desig-prop-value obj 'desig-props:name))))
        when (not (and distance-pregrasp distance-grasp))
          do (roslisp:ros-info (pr2 manip-pm)
                               "Pregrasp: ~a, Grasp: ~a"
                               distance-pregrasp distance-grasp)
             (return-from cost-function-ik-pose nil)
        summing (+ distance-pregrasp distance-grasp) into total-cost
        finally (return total-cost))))
    costme))

(defun arms-pose-distances (arms pose
                            &key
                              allowed-collision-objects
                              (constraint-aware nil)
                              (arms-offset-pose
                               (tf:make-identity-pose))
                              highlight-links)
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
                                        pose arms-offset-pose)
                  for pose-stamped = (tf:make-pose-stamped
                                      "/map" 0.0
                                      (tf:origin pose-offsetted)
                                      (tf:orientation pose-offsetted))
                  for publ = (publish-pose pose-stamped "/testpublisher2")
                  for distance = (reaching-length
                                  pose-stamped arm
                                  :constraint-aware constraint-aware
                                  :calc-euclidean-distance t
                                  :euclidean-target-link target-link
                                  :allowed-collision-objects
                                  allowed-collision-objects
                                  :highlight-links highlight-links)
                  when distance
                    collect (cons arm distance))))
      (when costme
        (ros-info (pr2 manip-pm) "Set of IK costs:")
        (loop for (arm . cost) in costme
              do (ros-info (pr2 manip-pm) "Arm = ~a, Cost = ~a" arm cost)))
      costme)))
