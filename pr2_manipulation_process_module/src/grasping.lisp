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
  (cl-transforms:make-pose
   (cl-transforms:make-3d-vector
    -0.20 0.0 0.0)
   (cl-transforms:euler->quaternion :ax (/ pi -2))))
(defparameter *pregrasp-top-slide-down-offset*
  (cl-transforms:make-pose
   (cl-transforms:make-3d-vector
    -0.20 0.10 0.0)
   (cl-transforms:euler->quaternion :ax (/ pi -2))))
(defparameter *pregrasp-pull-offset*
  (cl-transforms:make-pose
   (cl-transforms:make-3d-vector
    -0.28 -0.02 0)
   (cl-transforms:euler->quaternion :ax (/ pi 2)))) ;; Fix me
(defparameter *grasp-pull-offset*
  (cl-transforms:make-pose
   (cl-transforms:make-3d-vector
    -0.18 -0.02 0)
   (cl-transforms:euler->quaternion :ax (/ pi 2)))) ;; Fix me
(defparameter *grasp-offset*
  (cl-transforms:make-pose
   (cl-transforms:make-3d-vector
    -0.10 0.0 0.0)
   (cl-transforms:euler->quaternion :ax (/ pi -2))))
(defparameter *pre-putdown-offset*
  (cl-transforms:make-pose
   (cl-transforms:make-3d-vector
    0.0 0.0 0.04)
   (cl-transforms:euler->quaternion)))
(defparameter *putdown-offset*
  (cl-transforms:make-pose
   (cl-transforms:make-3d-vector
    0.0 0.0 0.02)
   (cl-transforms:euler->quaternion)))
(defparameter *unhand-offset*
  (cl-transforms:make-pose
   (cl-transforms:make-3d-vector
    -0.10 0.0 0.02)
   (cl-transforms:euler->quaternion)))
(defparameter *unhand-top-slide-down-offset*
  (cl-transforms:make-pose
   (cl-transforms:make-3d-vector
    0.0 0.0 0.10)
   (cl-transforms:euler->quaternion)))

;; Parking related poses
(defparameter *park-pose-left-default*
  (make-pose-stamped
   "torso_lift_link" (ros-time)
   (cl-transforms:make-3d-vector 0.1 0.45 0.3)
   (cl-transforms:euler->quaternion :ay (/ pi -2))))
(defparameter *park-pose-right-default*
  (make-pose-stamped
   "torso_lift_link" (ros-time)
   (cl-transforms:make-3d-vector 0.1 -0.45 0.3)
   (cl-transforms:euler->quaternion :ay (/ pi -2))))
(defparameter *park-pose-left-top-slide-down*
  (make-pose-stamped
   "base_link" (ros-time)
   (cl-transforms:make-3d-vector 0.3 0.5 1.3)
   (cl-transforms:euler->quaternion
    :ax 0 :ay (/ pi -2))))
(defparameter *park-pose-right-top-slide-down*
  (make-pose-stamped
   "base_link" (ros-time)
   (cl-transforms:make-3d-vector 0.3 -0.5 1.3)
   (cl-transforms:euler->quaternion
    :ax 0 :ay (/ pi -2))))

(defun absolute-handle (obj handle
                        &key (handle-offset-pose
                              (cl-transforms:make-identity-pose))
                          (reorient t))
  "Transforms the relative handle location `handle' of object `obj'
into the object's coordinate system and returns the appropriate
location designator. The optional parameter `handle-offset-pose' is
applied to the handle pose before the absolute object pose is
applied."
  (let* ((absolute-object-loc (desig-prop-value obj :at))
         (absolute-object-pose-stamped (cond (reorient
                                              (pose-pointing-away-from-base
                                               (desig-prop-value
                                                absolute-object-loc
                                                :pose)))
                                             (t (desig-prop-value
                                                 absolute-object-loc
                                                 :pose))))
         (relative-handle-loc (desig-prop-value handle :at))
         (relative-handle-pose (cl-transforms:transform-pose
                                (cl-transforms:pose->transform
                                 (reference relative-handle-loc))
                                handle-offset-pose))
         (pose-stamped (pose->pose-stamped
                        (frame-id absolute-object-pose-stamped)
                        (stamp absolute-object-pose-stamped)
                        (cl-transforms:transform-pose
                         (cl-transforms:pose->transform
                          absolute-object-pose-stamped)
                         relative-handle-pose))))
    (make-designator :object (loop for desc-elem in (description handle)
                                   when (eql (car desc-elem) :at)
                                     collect `(:at ,(make-designator
                                                    :location
                                                    `((:pose ,pose-stamped))))
                                   when (not (eql (car desc-elem) :at))
                                     collect desc-elem))))

(defun optimal-arm-handle-assignment (obj avail-arms avail-handles min-handles
                                      pregrasp-offset grasp-offset
                                      &key (max-handles
                                            (or (desig-prop-value
                                                 obj :max-handles)
                                                nil)))
  (optimal-arm-pose-assignment (mapcar (lambda (handle)
                                         (reference (desig-prop-value (cdr handle) :at)))
                                       avail-handles)
                               avail-arms min-handles pregrasp-offset grasp-offset
                               :obj obj :max-poses max-handles
                               :handles avail-handles))

(defun optimal-arm-pose-assignment (poses avail-arms min-arms
                                    pregrasp-offset grasp-offset
                                    &key obj (max-poses
                                              (or (and obj
                                                       (desig-prop-value
                                                        obj :max-handles))
                                                  nil))
                                      handles)
  (declare (ignore max-poses))
  (assert (= min-arms 1) () "Sorry, not more than one handle at a time right now.")
  (ros-info (pr2 manip-pm) "Opening grippers")
  (dolist (arm avail-arms)
    (open-gripper arm))
  (ros-info (pr2 manip-pm) "Calculating optimal grasp: ~a arms, ~a poses (min ~a)"
            (length avail-arms) (length poses) min-arms)
  (let* ((assignments
           (loop for arm in avail-arms
                 append (loop for i from 0 below (length poses)
                              as pose = (nth i poses)
                              as handle = (nth i handles)
                              as cost = (cost-function-ik-pose
                                         obj (list (list arm) (list pose))
                                         pregrasp-offset grasp-offset)
                              when cost
                                collect (make-instance
                                         'grasp-assignment
                                         :side arm
                                         :ik-cost cost
                                         :pose pose
                                         :handle-pair handle))))
         (sorted-assignments (sort assignments #'< :key #'ik-cost)))
    (ros-info (pr2 manip-pm) "Done calculating. Got ~a proper result(s)."
              (length sorted-assignments))
    sorted-assignments))

(defun cons->grasp-assignments (cons-cells)
  (mapcar (lambda (cons-cell)
            (cons->grasp-assignment cons-cell))
          cons-cells))

(defun cons->grasp-assignment (cons-cell &key handle cost)
  (make-instance 'grasp-assignment
                 :pose (car (cdr cons-cell))
                 :side (car cons-cell)
                 :handle-pair handle
                 :ik-cost cost
                 :grasp-type (cdr (cdr cons-cell))))

(defun make-grasp-assignment (&key side pose handle cost grasp-type
                                pregrasp-offset grasp-offset
                                gripper-offset close-radius)
  (make-instance 'grasp-assignment
                 :side side
                 :pose pose
                 :grasp-type grasp-type
                 :handle-pair handle
                 :ik-cost cost
                 :pregrasp-offset pregrasp-offset
                 :grasp-offset grasp-offset
                 :close-radius close-radius
                 :gripper-offset gripper-offset))

(defun cost-function-ik-pose (obj assignment pregrasp-offset grasp-offset
                              &key allowed-collision-objects)
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
  (loop for (arm . pose) in (mapcar #'cons
                                    (first assignment)
                                    (second assignment))
        as cost = (cost-reach-pose
                   obj arm pose pregrasp-offset grasp-offset
                   :allowed-collision-objects
                   allowed-collision-objects)
        when cost
          summing cost into total-cost
        finally (return total-cost)))

(defun cost-reach-pose (obj arm pose pregrasp-offset grasp-offset
                        &key allowed-collision-objects
                          only-reachable ignore-collisions-grasp)
  (let* ((pose (tf:copy-pose-stamped pose :stamp 0.0))
         (pregrasp-offset (or pregrasp-offset (tf:make-identity-pose)))
         (grasp-offset (or grasp-offset (tf:make-identity-pose)))
         (ignore-object (not obj))
         (distance-pregrasp
           (cond (only-reachable (is-pose-reachable
                                  pose arm
                                  :arm-offset-pose pregrasp-offset))
                 (t (cdr (assoc arm
                                (arms-pose-distances
                                 (list arm) pose
                                 :arms-offset-pose
                                 pregrasp-offset
                                 :highlight-links
                                 (links-for-arm-side arm)))))))
         (distance-grasp
           (when (and distance-pregrasp (> distance-pregrasp 0))
             (unless ignore-object
               (moveit:remove-collision-object
                (desig-prop-value obj :name)))
             (prog1
                 (cond (only-reachable (is-pose-reachable
                                        pose arm
                                        :arm-offset-pose grasp-offset
                                        :ignore-collisions ignore-collisions-grasp))
                       (t (cdr (assoc arm
                                      ;; ignore collisions missing here
                                      (arms-pose-distances
                                       (list arm) pose
                                       :arms-offset-pose
                                       grasp-offset
                                       :allowed-collision-objects
                                       allowed-collision-objects
                                       :highlight-links
                                       (links-for-arm-side arm))))))
               (unless ignore-object
                 (moveit:add-collision-object
                  (desig-prop-value obj :name)))))))
    (roslisp:ros-info (pr2 manip-pm) "Pregrasp: ~a, Grasp: ~a"
                      distance-pregrasp distance-grasp)
    (when (and distance-grasp (> distance-grasp 0))
      (+ distance-pregrasp distance-grasp))))

;; (defun cost-reach-pose (obj arm pose pregrasp-offset grasp-offset
;;                         &key allowed-collision-objects
;;                           only-reachable)
;;   (let* ((attached-objects (cram-moveit::get-robot-attached-objects))
;;          (distance-pregrasp
;;            (cond (only-reachable (is-pose-reachable
;;                                   pose arm
;;                                   :arm-offset-pose pregrasp-offset))
;;                  (t (cdr (assoc arm
;;                                 (arms-pose-distances
;;                                  (list arm) pose
;;                                  :arms-offset-pose
;;                                  pregrasp-offset
;;                                  :highlight-links
;;                                  (links-for-arm-side arm)))))))
;;          (distance-grasp (when (and distance-pregrasp (> distance-pregrasp 0))
;;                            (loop for object in attached-objects do
;;                              (moveit:detach-collision-object-from-link
;;                               (first object) (second object)))
;;                            (moveit:remove-collision-object
;;                             (desig-prop-value obj :name))
;;                            (prog1
;;                                (cond (only-reachable (is-pose-reachable
;;                                                       pose arm
;;                                                       :arm-offset-pose grasp-offset))
;;                                      (t (cdr (assoc arm
;;                                                     (arms-pose-distances
;;                                                      (list arm) pose
;;                                                      :arms-offset-pose
;;                                                      grasp-offset
;;                                                      :allowed-collision-objects
;;                                                      allowed-collision-objects
;;                                                      :highlight-links
;;                                                      (links-for-arm-side arm))))))
;;                              (moveit:add-collision-object
;;                               (desig-prop-value obj :name))
;;                              (loop for object in attached-objects do
;;                                (moveit:attach-collision-object-to-link
;;                                 (first object) (second object)))))))
;;     (roslisp:ros-info (pr2 manip-pm) "Pregrasp: ~a, Grasp: ~a"
;;                       distance-pregrasp distance-grasp)
;;     (when (and distance-grasp (> distance-grasp 0))
;;       (+ distance-pregrasp distance-grasp))))

(defun is-pose-reachable (pose arm &key arm-offset-pose ignore-collisions)
  (let ((pose (tf:copy-pose-stamped
               (cond (arm-offset-pose (relative-pose pose arm-offset-pose))
                     (t pose))
               :stamp 0.0)))
    (roslisp:publish (roslisp:advertise "/testpose" "geometry_msgs/PoseStamped")
                     (to-msg pose))
    (cpl:with-failure-handling
        ((moveit:no-ik-solution (f)
           (declare (ignore f))
           (return 0)))
      (when (moveit:compute-ik
             (link-name arm)
             (ecase arm
               (:left "left_arm")
               (:right "right_arm"))
             pose
             :avoid-collisions (not ignore-collisions))
        1))))

(defun arms-pose-distances (arms pose
                            &key
                              allowed-collision-objects
                              (arms-offset-pose
                               (cl-transforms:make-identity-pose))
                              highlight-links)
  (loop for arm in arms
        for distance = (reaching-length
                        (relative-pose pose arms-offset-pose) arm
                        :allowed-collision-objects
                        allowed-collision-objects
                        :highlight-links highlight-links)
        when distance
          collect (progn
                    (ros-info (pr2 manip-pm) "Arm = ~a, Cost = ~a" arm distance)
                    (cons arm distance))))
