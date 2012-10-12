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

(in-package :pr2-manip-pm)

(defparameter *handle-pregrasp-offset-pose*
  (tf:make-pose
   (tf:make-3d-vector 0.15 0.0 0.0)
   (tf:euler->quaternion :az pi :ax (/ pi 2)))
  "Specifies the gripper pose relative to the respective handle
  coordinate system (including it's origin and rotation) when going
  into pregrasp.")
(defparameter *handle-grasp-offset-pose*
  (tf:make-pose
   (tf:make-3d-vector 0.12 0.0 0.0)
   (tf:euler->quaternion :az pi :ax (/ pi 2)))
  "Specifies the gripper pose relative to the respective handle
  coordinate system (including it's origin and rotation) when going
  into grasp.")

(defun grab-object-with-handles-constraint-aware (obj side obstacles
                                                  &key
                                                    obj-as-obstacle)
  (clear-collision-objects)
  (dolist (obstacle (cut:force-ll obstacles))
    (register-collision-object obstacle))
  (register-collision-object obj)
  ;; NOTE(winkler): Check for the constraint-aware IK service. At the
  ;; moment, register-collision-object is not really implemented.
  (grab-object-with-handles obj side :constraint-aware t))

(defun grab-object-with-handles (obj side &key constraint-aware)
  "Grasp an object `obj' on one of its handles with the specified
gripper side `side'. This includes going into pregrasp for the nearest
handle, opening the gripper, going into the grasp position, closing
the gripper and lifting the object by 0.2m by default."
  (let* ((handles (desig-prop-values obj 'handle)))
    (assert (> (length handles) 0)
            () "Object ~a needs at least one handle." obj)
    (let* ((nearest-handle-data
             (nearest-handle
              obj
              :side side
              :handle-offset-pose *handle-pregrasp-offset-pose*
              :constraint-aware constraint-aware))
           (nearest-side (first nearest-handle-data))
           (nearest-handle (second nearest-handle-data)))
      (declare (ignore nearest-side))
      (cond (nearest-handle
             (let ((handle-radius (or (desig-prop-value
                                       nearest-handle
                                       'radius)
                                      0.0)))
               (roslisp:ros-info (pr2-manipulation-process-module)
                                 "Going into pregrasp for handled object")
               (pregrasp-handled-object-with-relative-location
                obj
                side
                nearest-handle
                :constraint-aware constraint-aware)
               (roslisp:ros-info (pr2-manipulation-process-module)
                                 "Opening gripper")
               (open-gripper side :position (+ handle-radius 0.02))
               (roslisp:ros-info (pr2-manipulation-process-module)
                                 "Going into grasp for handled object")
               ;; NOTE(winkler): The grasp itself should not be
               ;; constraint-aware as we are already near the object
               ;; (no obstacles between gripper and object assumed)
               ;; and we need to get real close to the object with the
               ;; gripper. Having this function constraint-aware would
               ;; break the grasping.
               (grasp-handled-object-with-relative-location
                obj
                side
                nearest-handle)
               (roslisp:ros-info (pr2-manipulation-process-module)
                                 "Closing gripper")
               (close-gripper side :position handle-radius)
               (check-valid-gripper-state
                side
                :min-position (- handle-radius 0.01)))
             (roslisp:ros-info (pr2-manip process-module)
                               "Attaching object to gripper")
             (plan-knowledge:on-event
              (make-instance
               'plan-knowledge:object-attached
               :object obj
               :link (ecase side
                       (:right "r_gripper_r_finger_tip_link")
                       (:left "l_gripper_r_finger_tip_link"))
               :side side)))
            (t
             (cpl:fail 'manipulation-pose-unreachable))))))

(defun taxi-handled-object (obj side handle
                            &key (relative-gripper-pose
                                  (tf:make-identity-pose))
                              constraint-aware)
  "Commutes the arm to a certain absolute pose. The target pose is
determined through the absolute object pose of object `obj', the
relative object handle location `relative-handle-loc' and the relative
gripper pose `relative-gripper-pose' w.r.t. the handle. The relative
gripper pose defaults to an identity pose."
  (let* ((absolute-pose-map (object-handle-absolute
                             obj
                             handle
                             :handle-offset-pose relative-gripper-pose))
         ;; NOTE(winkler): We're transforming into the tf-frame
         ;; "torso_lift_link" here due to the fact that get-ik
         ;; (service node constraint_awake_ik) does not transform from
         ;; the map-frame. The constructor of the class does not
         ;; create a tf listener. Therefore, the goal has to be
         ;; specified in the ik root.
         (absolute-pose (tf:transform-pose *tf*
                                           :pose absolute-pose-map
                                           :target-frame "torso_lift_link"))  
         ;; This makes reach the carry-pose impossible
         ;; atm. :seed-state (calc-seed-state-elbow-up side))))
         (seed-state (calc-seed-state-elbow-up
                      side
                      :elbow-up nil
                      :elbow-out nil))
         (move-ik (if constraint-aware
                      (get-constraint-aware-ik side
                                               absolute-pose
                                               :seed-state seed-state)
                      (get-ik side absolute-pose :seed-state seed-state))))
    (unless move-ik (cpl:fail
                     'cram-plan-failures:manipulation-pose-unreachable))
    (let ((move-trajectory (ik->trajectory (first move-ik) :duration 5.0)))
      (unless move-trajectory (cpl:fail
                               'cram-plan-failures:manipulation-failed))
      (nth-value 1 (execute-arm-trajectory side move-trajectory)))))

(defun grasp-handled-object-with-relative-location (obj side handle
                                                    &key constraint-aware)
  "Moves the gripper side `side' into the grasp position with respect
to the object's `obj' handle `handle'."
  (taxi-handled-object
   obj side handle :relative-gripper-pose *handle-grasp-offset-pose*
                   :constraint-aware constraint-aware))

(defun pregrasp-handled-object-with-relative-location (obj side handle
                                                       &key constraint-aware)
  "Moves the gripper side `side' into the pregrasp position with
respect to the object's `obj' handle `handle'."
  (taxi-handled-object
   obj side handle :relative-gripper-pose *handle-pregrasp-offset-pose*
                   :constraint-aware constraint-aware))

(defun object-handle-absolute (obj handle
                               &key (handle-offset-pose
                                     (tf:make-identity-pose)))
  "Transforms the relative handle location `handle' of object `obj'
into the object's coordinate system and returns the appropriate
location designator. The optional parameter `handle-offset-pose' is
applied to the handle pose before the absolute object pose is
applied."
  (let* ((absolute-object-loc (desig-prop-value obj 'at))
         (absolute-object-pose-stamped (reference absolute-object-loc))
         (relative-handle-loc (desig-prop-value handle 'at))
         (relative-handle-pose (cl-transforms:transform-pose
                                (tf:pose->transform
                                 (reference relative-handle-loc))
                                handle-offset-pose)))
    (tf:pose->pose-stamped
     (tf:frame-id absolute-object-pose-stamped)
     (tf:stamp absolute-object-pose-stamped)
     (cl-transforms:transform-pose
      (tf:pose->transform
       absolute-object-pose-stamped)
      relative-handle-pose))))

(defun nearest-handle (obj &key side (handle-offset-pose
                                      (tf:make-identity-pose))
                             constraint-aware)
  "Get the nearest handle location designator on object `obj'. If the
parameter `side' is set, the nearest handle for this side is
determined. If it is left out (e.g. has a value of NIL), the nearest
handle in respect of both sides (:left, :right) is calculated. A list
of the form `(side handle)' is returned. The optional parameter
`handle-offset-pose' can be used to specify an offset to the
respective handles in their respective coordinate system."
  (cond (side
         (let ((nearest-handle-data 
                 (nearest-handle-for-side
                  obj
                  side
                  :handle-offset-pose handle-offset-pose
                  :constraint-aware constraint-aware)))
           (list side (second nearest-handle-data))))
        (t
         (let* ((nearest-left (nearest-handle-for-side
                               obj
                               :left
                               :handle-offset-pose handle-offset-pose
                               :constraint-aware constraint-aware))
                (nearest-right (nearest-handle-for-side
                                obj
                                :right
                                :handle-offset-pose handle-offset-pose
                                :constraint-aware constraint-aware))
                (distance-left (first nearest-left))
                (distance-right (first nearest-right))
                (handle-left (second nearest-left))
                (handle-right (second nearest-right)))
           (cond ((and handle-left handle-right
                       distance-left distance-right)
                  (if (< distance-left distance-right)
                      (list :left handle-left)
                      (list :right handle-right)))
                 (handle-left (list :left handle-left))
                 (handle-right (list :right handle-right))
                 (t (roslisp::ros-warn (pr2-manip-pm handled-objects)
                                       "No nearest handle found.")
                    (list nil nil)))))))

(defun nearest-handle-for-side (obj side &key (handle-offset-pose
                                               (tf:make-identity-pose))
                                           constraint-aware)
  "Calculate the closest handle (in terms of joint-space) for side
`side'. Returns a list in the form of (distance handle) for the found
nearest handle and (-1 NIL) if no reachable handles were found. The
optional parameter `handle-offset-pose' can be used to specify an
offset to the respective handles in their respective coordinate
system."
  (let ((lowest-distance nil)
        (nearest-handle nil)
        (handles (desig-prop-values obj 'handle)))
    (loop for handle in handles
          ;; NOTE(winkler): Both, the distance with the pregrasp pose
          ;; included and the one without the pregrasp pose are
          ;; checked here for valid IK solutions. There is no sense in
          ;; marking a handle as "reachable" just because the pregrasp
          ;; pose is reachable.
          for distance-with-offset = (reaching-length
                                      (object-handle-absolute
                                       obj
                                       handle
                                       :handle-offset-pose
                                       handle-offset-pose)
                                      side
                                      :constraint-aware
                                      constraint-aware
                                      :calc-euclidean-distance t)
          for distance-without-offset = (reaching-length
                                         (object-handle-absolute
                                          obj
                                          handle)
                                         side
                                         :constraint-aware
                                         constraint-aware
                                         :calc-euclidean-distance t)
          when (and distance-with-offset distance-without-offset
                    (or (not lowest-distance)
                        (and distance-with-offset
                             (< distance-with-offset
                                lowest-distance))))
            do (assert handle)
               (setf lowest-distance distance-with-offset)
               (setf nearest-handle handle))
    (list lowest-distance nearest-handle)))
