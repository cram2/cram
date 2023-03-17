;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :giskard)

(defparameter *arm-convergence-delta-xy* 0.025 ;; 0.005
  "in meters")
(defparameter *arm-convergence-delta-theta* 0.5 ;; 0.1
  "in radiants, about 30 degrees")
(defparameter *arm-convergence-delta-joint* 0.17
  "in radiants, about 10 degrees")
(defparameter *arm-max-velocity-slow-xy* 0.1
  "in m/s")
(defparameter *arm-max-velocity-slow-theta* 0.5
  "in rad/s")


;;@author Felix Krause
(defun make-custom-return-grasp-constraint (knob-pose open-drawer
                                     &key avoid-collisions-not-much)
  ;; (declare  (type cl-transforms-stamped:pose-stamped goal-pose))
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "MoveDrawer"
   :parameter_value_pair
   (giskard::alist->json-string
    `(("knob_pose"
       . (("message_type" . "geometry_msgs/PoseStamped")
          ("message" . ,(giskard::to-hash-table knob-pose))))
      ("open_drawer"
         . ,open-drawer)
      
      ,@(if avoid-collisions-not-much
            `(("weight" . ,(roslisp-msg-protocol:symbol-code
                           'giskard_msgs-msg:constraint
                           :weight_above_ca))
              (("weight" . (roslisp-msg-protocol:symbol-code
                            'giskard_msgs-msg:constraint
                            :weight_below_ca)))))))))

;;@author Felix Krause
(defun make-custom-grasp-constraint (box-pose box-name;; box-size ;; grasp-type
                                     &key avoid-collisions-not-much)
  ;; (break)
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "GraspObject"
   :parameter_value_pair
   (giskard::alist->json-string
    `(("box_pose"
       . (("message_type" . "geometry_msgs/PoseStamped")
          ("message" . ,(giskard::to-hash-table box-pose))))
      ("box_name"
         . ,box-name)
      
      ,@(if avoid-collisions-not-much
            `(("weight" . ,(roslisp-msg-protocol:symbol-code
                           'giskard_msgs-msg:constraint
                           :weight_above_ca))
              (("weight" . (roslisp-msg-protocol:symbol-code
                            'giskard_msgs-msg:constraint
                            :weight_below_ca)))))))))

;;@author Luca Krohm
(defun make-reach-constraint (object-pose object-name object-size
                              &key avoid-collisions-not-much)
  "Receives parameters used by manipulation. Creates Constraint of the type GraspObject which is a classname inside the manipulation code, which is responsible for 'reaching'"
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "GraspObject"
   :parameter_value_pair
   (giskard::alist->json-string
    `(("object_pose"
       . (("message_type" . "geometry_msgs/PoseStamped")
          ("message" . ,(giskard::to-hash-table object-pose))))
      ("object_name"
       . ,object-name)
      ("object_size"
       . (("message_type" . "geometry_msgs/Vector3")
          ("message" . ,(giskard::to-hash-table object-size))))
      
      ,@(if avoid-collisions-not-much
            `(("weight" . ,(roslisp-msg-protocol:symbol-code
                           'giskard_msgs-msg:constraint
                           :weight_above_ca))
              (("weight" . (roslisp-msg-protocol:symbol-code
                            'giskard_msgs-msg:constraint
                            :weight_below_ca)))))))))

;;@author Luca Krohm
(defun make-lift-constraint (object-name
                             &key avoid-collisions-not-much)
  "Receives parameters used by manipulation. Creates Constraint of the type LiftObject which is a classname inside the manipulation code, which is responsible for 'lifting'"  
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "LiftObject"
   :parameter_value_pair
   (giskard::alist->json-string
    `(("object_name"
         . ,object-name)      
      ,@(if avoid-collisions-not-much
            `(("weight" . ,(roslisp-msg-protocol:symbol-code
                           'giskard_msgs-msg:constraint
                           :weight_above_ca))
              (("weight" . (roslisp-msg-protocol:symbol-code
                            'giskard_msgs-msg:constraint
                            :weight_below_ca)))))))))

;;@author Luca Krohm
(defun make-retract-constraint (object-name
                                     &key avoid-collisions-not-much)
  "Receives parameters used by manipulation. Creates Constraint of the type Retracting which is a classname inside the manipulation code, which is responsible for 'retracting'"
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "Retracting"
   :parameter_value_pair
   (giskard::alist->json-string
    `(("object_name"
         . ,object-name)      
      ,@(if avoid-collisions-not-much
            `(("weight" . ,(roslisp-msg-protocol:symbol-code
                           'giskard_msgs-msg:constraint
                           :weight_above_ca))
              (("weight" . (roslisp-msg-protocol:symbol-code
                            'giskard_msgs-msg:constraint
                            :weight_below_ca)))))))))

(defun make-prepare-place-constraint (target-pose object-height
                                     &key avoid-collisions-not-much)
  "Receives parameters used by manipulation. Creates Constraint of the type PreparePlacing which is a classname inside the manipulation code, which is responsible for 'preparing-placing'"
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "PreparePlacing"
   :parameter_value_pair
   (giskard::alist->json-string
    `(("target_pose"
       . (("message_type" . "geometry_msgs/PoseStamped")
          ("message" . ,(giskard::to-hash-table target-pose))))
      ("object_height"
       . ,object-height)
      
      ,@(if avoid-collisions-not-much
            `(("weight" . ,(roslisp-msg-protocol:symbol-code
                           'giskard_msgs-msg:constraint
                           :weight_above_ca))
              (("weight" . (roslisp-msg-protocol:symbol-code
                            'giskard_msgs-msg:constraint
                            :weight_below_ca)))))))))

(defun make-place-constraint (target-pose object-name object-height
                                     &key avoid-collisions-not-much)
  "Receives parameters used by manipulation. Creates Constraint of the type PlaceObject which is a classname inside the manipulation code, which is responsible for 'placing'"
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "PlaceObject"
   :parameter_value_pair
   (giskard::alist->json-string
    `(("target_pose"
       . (("message_type" . "geometry_msgs/PoseStamped")
          ("message" . ,(giskard::to-hash-table target-pose))))
      ("object_name"
       . ,object-name)
      ("object_height"
       . ,object-height)
      
      ,@(if avoid-collisions-not-much
            `(("weight" . ,(roslisp-msg-protocol:symbol-code
                           'giskard_msgs-msg:constraint
                           :weight_above_ca))
              (("weight" . (roslisp-msg-protocol:symbol-code
                            'giskard_msgs-msg:constraint
                            :weight_below_ca)))))))))

(defun make-arm-cartesian-action-goal (left-pose right-pose
                                       pose-base-frame collision-mode
                                       &key
                                         collision-object-b
                                         collision-object-b-link
                                         collision-object-a
                                         prefer-base allow-base straight-line
                                         align-planes-left align-planes-right
                                         unmovable-joints
                                         precise-tracking
                                         box-pose
                                         object-pose
                                         target-pose
                                         action-type
                                         object-size
                                         object-height
                                         knob-pose
                                         open-drawer)
  (declare (type (or null cl-transforms-stamped:pose-stamped) left-pose right-pose)
           (type (or null string) pose-base-frame)
           (type boolean prefer-base straight-line
                 align-planes-left align-planes-right precise-tracking)
           (type (or null list) unmovable-joints))
  (let ((arms (append (when left-pose '(:left))
                      (when right-pose '(:right)))))
    (make-giskard-goal
     :constraints (list
                   (make-avoid-joint-limits-constraint
                    :joint-list (append (when left-pose
                                          (cut:var-value
                                           '?joints
                                           (car
                                            (prolog:prolog
                                             `(and (rob-int:robot ?robot-name)
                                                   (rob-int:arm-joints
                                                    ?robot-name :left ?joints))))))
                                        (when right-pose
                                          (cut:var-value
                                           '?joints
                                           (car
                                            (prolog:prolog
                                             `(and (rob-int:robot ?robot-name)
                                                   (rob-int:arm-joints
                                                    ?robot-name :right ?joints))))))))
                   (when allow-base
                     (make-prefer-base-constraint
                      :base-weight (if prefer-base
                                       *prefer-base-low-cost*
                                       *allow-base-high-cost*)))
                   (when align-planes-left
                     (make-align-planes-constraint
                      pose-base-frame
                      cram-tf:*robot-left-tool-frame*
                      (cl-transforms-stamped:make-vector-stamped
                       cram-tf:*robot-base-frame* 0.0
                       (cl-transforms:make-3d-vector 0 0 1))
                      (cl-transforms-stamped:make-vector-stamped
                       cram-tf:*robot-base-frame* 0.0
                       (cl-transforms:make-3d-vector 0 0 1))))
                   (when align-planes-right
                     (make-align-planes-constraint
                      pose-base-frame
                      cram-tf:*robot-right-tool-frame*
                      (cl-transforms-stamped:make-vector-stamped
                       cram-tf:*robot-base-frame* 0.0
                       (cl-transforms:make-3d-vector 0 0 1))
                      (cl-transforms-stamped:make-vector-stamped
                       cram-tf:*robot-base-frame* 0.0
                       (cl-transforms:make-3d-vector 0 0 1))))
                   (when unmovable-joints
                     (make-unmovable-joints-constraint unmovable-joints))
                   ;; (make-base-velocity-constraint
                   ;;  *base-max-velocity-slow-xy* *base-max-velocity-slow-theta*)
                   ;; (make-head-pointing-at-hand-constraint
                   ;;  (if left-pose
                   ;;      :left
                   ;;      :right))
                   (when (eq (rob-int:get-robot-name) :tiago-dual)
                     (make-diffdrive-cartesian-goal-arm-constraint
                      (if left-pose
                          cram-tf:*robot-left-wrist-frame*
                          cram-tf:*robot-right-wrist-frame*)))
                   (when precise-tracking
                     (make-enable-velocity-trajectory-tracking-constraint))
                   (when left-pose
                     (make-cartesian-constraint
                      pose-base-frame
                      cram-tf:*robot-left-wrist-frame*
                      ;;cram-tf:*robot-left-tool-frame*
                      left-pose
                      :straight-line straight-line
                      :avoid-collisions-much nil))
                   (when right-pose
                     (make-cartesian-constraint
                      pose-base-frame
                      cram-tf:*robot-right-wrist-frame*
                      ;;cram-tf:*robot-right-tool-frame*
                      right-pose
                      :straight-line straight-line
                      :avoid-collisions-much nil))
                   ;;;;
                   ;; Constraints for motions
                   (when (eq action-type 'reach)
                     (make-reach-constraint object-pose "pringles" object-size ))
                   (when (eq action-type 'lift)
                     (make-lift-constraint "pringles" ))
                   (when (eq action-type 'retract)
                      (make-retract-constraint "pringles" ))
                   (when (eq action-type 'prepare-place)
                     (make-prepare-place-constraint target-pose object-height))
                   (when (eq action-type 'place)
                     (make-place-constraint target-pose "pringles" object-height))
                   ;;;;
                   
                   (when box-pose
                     (make-custom-grasp-constraint box-pose "pringles" ))
                   (when knob-pose
                     (make-custom-return-grasp-constraint knob-pose open-drawer))

                    )
     :collisions (ecase collision-mode
                   (:avoid-all (make-avoid-all-collision))
                   (:allow-all (make-allow-all-collision))
                   (:allow-hand (alexandria:flatten
                                 (list
                                  (make-allow-hand-collision
                                   arms collision-object-b
                                   collision-object-b-link)
                                  (make-allow-hand-collision
                                   arms (rob-int:get-environment-name)))))
                   (:allow-fingers (alexandria:flatten
                                    (list
                                     (make-allow-fingers-collision
                                      arms collision-object-b
                                      collision-object-b-link)
                                     (make-allow-fingers-collision
                                      arms (rob-int:get-environment-name)))))
                   (:allow-arm (alexandria:flatten
                                (list
                                 (make-allow-arm-collision
                                  arms collision-object-b
                                  collision-object-b-link)
                                 (make-allow-arm-collision
                                  arms (rob-int:get-environment-name)))))
                   ;; TODO: this should allow collision between attached and environment
                   (:allow-attached (make-avoid-all-collision))))))

(defun make-arm-joint-action-goal (joint-state-left joint-state-right
                                   align-planes-left align-planes-right
                                   &key try-harder)
  (declare (type list joint-state-left joint-state-right)
           (type boolean align-planes-left align-planes-right))
  (make-giskard-goal
   :constraints (list
                 ;; (make-ee-velocity-constraint
                 ;;  :left
                 ;;  (if try-harder
                 ;;      (/ *arm-max-velocity-slow-xy* 3.0)
                 ;;      *arm-max-velocity-slow-xy*)
                 ;;  (if try-harder
                 ;;      (/ *arm-max-velocity-slow-theta* 3.0)
                 ;;      *arm-max-velocity-slow-theta*))
                 ;; (make-ee-velocity-constraint
                 ;;  :right
                 ;;  (if try-harder
                 ;;      (/ *arm-max-velocity-slow-xy* 3.0)
                 ;;      *arm-max-velocity-slow-xy*)
                 ;;  (if try-harder
                 ;;      (/ *arm-max-velocity-slow-theta* 3.0)
                 ;;      *arm-max-velocity-slow-theta*))
                 (make-cartesian-constraint
                  cram-tf:*odom-frame* cram-tf:*robot-base-frame*
                  (cl-transforms-stamped:pose->pose-stamped
                   cram-tf:*robot-base-frame* 0.0
                   (cl-transforms:make-identity-pose))
                  :max-velocity *base-max-velocity-slow-xy*
                  :avoid-collisions-much nil)
                 (when align-planes-left
                   (make-align-planes-tool-frame-constraint
                    :left
                    (cl-transforms-stamped:make-vector-stamped
                     cram-tf:*robot-base-frame* 0.0
                     (cl-transforms:make-3d-vector 0 0 1))
                    (cl-transforms-stamped:make-vector-stamped
                     cram-tf:*robot-base-frame* 0.0
                     (cl-transforms:make-3d-vector 0 0 1))))
                 (when align-planes-right
                   (make-align-planes-tool-frame-constraint
                    :right
                    (cl-transforms-stamped:make-vector-stamped
                     cram-tf:*robot-base-frame* 0.0
                     (cl-transforms:make-3d-vector 0 0 1))
                    (cl-transforms-stamped:make-vector-stamped
                     cram-tf:*robot-base-frame* 0.0
                     (cl-transforms:make-3d-vector 0 0 1)))))
   :joint-constraints (list (make-joint-constraint joint-state-left)
                            (make-joint-constraint joint-state-right))
   :collisions (list (make-avoid-all-collision))))



(defun ensure-arm-cartesian-goal-input (frame goal-pose arm)
  (when goal-pose
    (let* ((tool-pose-in-correct-base-frame
             (cram-tf:ensure-pose-in-frame goal-pose frame))
           (tool-frame
             (if (eq arm :left)
                 cram-tf:*robot-left-tool-frame*
                 cram-tf:*robot-right-tool-frame*))
           (tool-transform-in-correct-base-frame
             (cram-tf:pose-stamped->transform-stamped
              tool-pose-in-correct-base-frame
              tool-frame))
           (wrist-frame
             (if (eq arm :left)
                 cram-tf:*robot-left-wrist-frame*
                 cram-tf:*robot-right-wrist-frame*))
           (ee-P-tcp
             (cut:var-value
              '?ee-P-tcp
              (car
               (prolog:prolog
                `(and (rob-int:robot ?robot-name)
                      (rob-int:tcp-in-ee-pose ?robot-name ?ee-P-tcp))))))
           (tool-T-wrist
             (cl-transforms-stamped:transform->transform-stamped
              tool-frame
              wrist-frame
              0.0
              (cl-transforms:transform-inv
               (cl-transforms:pose->transform ee-P-tcp))))
           (wrist-pose-in-correct-base-frame
             (cram-tf:multiply-transform-stampeds
              frame wrist-frame
              tool-transform-in-correct-base-frame
              tool-T-wrist
              :result-as-pose-or-transform :pose)))
      ;; wrist-pose-in-correct-base-frame
      tool-pose-in-correct-base-frame)))

(defun ensure-arm-joint-goal-input (goal-configuration arm)
  (if (and (listp goal-configuration)
           (or (= (length goal-configuration) 7)
               (= (length goal-configuration) 6)))
      (get-arm-joint-names-and-positions-list arm goal-configuration)
      (progn (roslisp:ros-warn (giskard joint)
                               "Joint goal ~a was not a list of 7 (or 6). Ignoring."
                               goal-configuration)
             (get-arm-joint-names-and-positions-list arm))))

(defun ensure-arm-cartesian-goal-reached (goal-pose goal-frame)
  (when goal-pose
    (multiple-value-bind (converged delta-xy delta-theta)
        (cram-tf:tf-frame-converged
         goal-frame goal-pose
         *arm-convergence-delta-xy* *arm-convergence-delta-theta*)
     (unless converged
       (make-instance 'common-fail:manipulation-goal-not-reached
         :description (format nil "Giskard did not converge to goal:~%~
                                   ~a should have been at ~a.~%
                                   Delta-xy: ~a, delta-theta: ~a."
                              goal-frame goal-pose
                              delta-xy delta-theta))))))

(defun ensure-arm-joint-goal-reached (goal-configuration arm)
  (when goal-configuration
    (let ((current-angles
            (cram-tf:normalize-joint-angles
             (second (get-arm-joint-names-and-positions-list arm))))
          (goal-angles
            (cram-tf:normalize-joint-angles
             (mapcar #'second goal-configuration))))
      (unless (cram-tf:values-converged
               current-angles goal-angles *arm-convergence-delta-joint*)
        (make-instance 'common-fail:manipulation-goal-not-reached
          :description (format nil "Giskard did not converge to goal:~%~a (~a)~%~
                                    should have been at~%~a~%with delta-joint of ~a."
                               arm current-angles goal-angles
                               *arm-convergence-delta-joint*))))))

(defun call-arm-cartesian-action (&key
                                    action-timeout
                                    goal-pose-left goal-pose-right
                                    pose-base-frame
                                    collision-mode
                                    collision-object-b collision-object-b-link
                                    collision-object-a
                                    move-base prefer-base straight-line
                                    align-planes-left align-planes-right
                                    unmovable-joints
                                    precise-tracking
                                    box-pose
                                    action-type
                                    target-pose
                                    object-size
                                    object-height
                                    ;; grasp-type
                                    knob-pose
                                    open-drawer
                                    object-pose)
  (declare (type (or number null) action-timeout)
           (type (or cl-transforms-stamped:pose-stamped null)
                 goal-pose-left goal-pose-right)
           (type (or string null) pose-base-frame)
           (type boolean move-base prefer-base straight-line precise-tracking
                 align-planes-left align-planes-right)
           (type (or list null) unmovable-joints))

  ;; (unless (or goal-pose-left goal-pose-right)
  ;;   (roslisp:ros-info (giskard cart) "Got an empty goal...")
  ;;   ;; return NIL as observation if the goal is empty
  ;;   (return-from call-arm-cartesian-action))

  (when prefer-base
    (setf move-base T))
  (if (and move-base (not pose-base-frame))
      (setf pose-base-frame cram-tf:*odom-frame*)
      (setf pose-base-frame cram-tf:*robot-base-frame*))
  ;; (setf goal-pose-left
  ;;       (ensure-arm-cartesian-goal-input pose-base-frame goal-pose-left :left))
  ;; (setf goal-pose-right
  ;;       (ensure-arm-cartesian-goal-input pose-base-frame goal-pose-right :right))

  ;; (cram-tf:visualize-marker
  ;;  (list goal-pose-left goal-pose-right)
  ;;  :r-g-b-list '(1 0 1))

  (call-action
   :action-goal (make-arm-cartesian-action-goal
                 goal-pose-left goal-pose-right
                 pose-base-frame
                 collision-mode
                 :collision-object-b collision-object-b
                 :collision-object-b-link collision-object-b-link
                 :collision-object-a collision-object-a
                 :allow-base move-base
                 :prefer-base prefer-base
                 :straight-line straight-line
                 :align-planes-left align-planes-left
                 :align-planes-right align-planes-right
                 :unmovable-joints unmovable-joints
                 :precise-tracking precise-tracking
                 :box-pose box-pose
                 :object-pose object-pose
                 :action-type action-type
                 :target-pose target-pose
                 :object-size object-size
                 :object-height object-height
                 ;; :grasp-type grasp-type
                 :knob-pose knob-pose
                 :open-drawer open-drawer)
   :action-timeout action-timeout
   ;; :check-goal-function (lambda (result status)
   ;;                        (declare (ignore result status))
   ;;                        (or (ensure-arm-cartesian-goal-reached
   ;;                             goal-pose-left cram-tf:*robot-left-tool-frame*)
   ;;                            (ensure-arm-cartesian-goal-reached
   ;;                             goal-pose-right cram-tf:*robot-right-tool-frame*)))
   ))



(defun call-arm-joint-action (&key
                                action-timeout
                                goal-configuration-left goal-configuration-right
                                align-planes-left align-planes-right
                                avoid-collisions-not-much)
  (declare (type (or list null) goal-configuration-left goal-configuration-right)
           (type (or number null) action-timeout)
           (type boolean avoid-collisions-not-much))

  (let ((joint-state-left
          (ensure-arm-joint-goal-input goal-configuration-left :left))
        (joint-state-right
          (ensure-arm-joint-goal-input goal-configuration-right :right)))

    (call-action
     :action-goal (make-arm-joint-action-goal
                   joint-state-left joint-state-right
                   align-planes-left align-planes-right
                   :try-harder avoid-collisions-not-much)
     :action-timeout action-timeout
     :check-goal-function (lambda (result status)
                            (declare (ignore result status))
                            (or (ensure-arm-joint-goal-reached
                                 goal-configuration-left :left)
                                (ensure-arm-joint-goal-reached
                                 goal-configuration-right :right))))))


