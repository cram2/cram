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

(in-package :pr2-ik-fk)

(defparameter *use-arm-kinematics-interface* nil
  "The default interface is moveit interface with one compute_ik for both arms.
When `*use-arm-kinematics-interface*' is set, use separate services for each arm.")


(defparameter *ik-service-names* '(:moveit "moveit/compute_ik"
                                   :left "pr2_left_arm_kinematics/get_ik"
                                   :right "pr2_right_arm_kinematics/get_ik"))

(defvar *ik-persistent-services* '(:moveit nil :left nil :right nil)
  "a persistent service for ik requests")

(defun init-ik-persistent-service (moveit-or-left-or-right)
  (declare (type keyword moveit-or-left-or-right))
  "Initializes one of the corresponding *ik-persistent-services*"
  (let ((service-name (getf *ik-service-names* moveit-or-left-or-right)))
    (loop until (roslisp:wait-for-service service-name 5)
          do (roslisp:ros-info (ik-service) "Waiting for ~a." service-name))
    (prog1 (setf (getf *ik-persistent-services* moveit-or-left-or-right)
                 (make-instance 'roslisp:persistent-service
                   :service-name service-name
                   :service-type 'moveit_msgs-srv:getpositionik))
      (roslisp:ros-info (ik-service) "~a client created." service-name))))

(defun get-ik-persistent-service (moveit-or-left-or-right)
  (declare (type keyword moveit-or-left-or-right))
  (let ((service (getf *ik-persistent-services* moveit-or-left-or-right)))
    (if (and service (roslisp:persistent-service-ok service))
        service
        (init-ik-persistent-service moveit-or-left-or-right))))

(defun destroy-ik-persistent-services ()
  (mapcar (lambda (moveit-or-left-or-right)
            (let ((service (getf *ik-persistent-services* moveit-or-left-or-right)))
              (when service (roslisp:close-persistent-service service)))
            (setf (getf *ik-persistent-services* moveit-or-left-or-right) nil))
          '(:moveit :left :right)))

(roslisp-utilities:register-ros-cleanup-function destroy-ik-persistent-services)


(defun get-pr2-arm-joints-list (arm)
  (cut:var-value '?joints
                 (car (prolog:prolog `(cram-robot-interfaces:arm-joints
                                       cram-pr2-description:pr2 ,arm ?joints)))))

(defun get-pr2-ee-link (arm)
  (cut:var-value '?link
                 (car (prolog:prolog `(cram-robot-interfaces:end-effector-link
                                       cram-pr2-description:pr2 ,arm ?link)))))

(defun get-pr2-planning-group (arm)
  (cut:var-value '?group
                 (car (prolog:prolog `(cram-robot-interfaces:planning-group
                                       cram-pr2-description:pr2 ,arm ?group)))))


(defun make-zero-seed-state (left-or-right)
  (let* ((joint-names (map 'vector #'identity (get-pr2-arm-joints-list left-or-right)))
         (zero-vector (apply #'vector (make-list
                                       (length joint-names)
                                       :initial-element 0.0))))
    (roslisp:make-message
     "sensor_msgs/JointState"
     name joint-names
     position zero-vector
     velocity zero-vector
     effort zero-vector)))

;; (defun make-bullet-current-seed-state (left-or-right)
;;   (let* ((joint-names-vector (get-ik-solver-joints left-or-right))
;;          (joint-names (map 'list #'identity joint-names-vector))
;;          (joint-states (map 'vector #'joint-state-position (joint-states joint-names)))
;;          (zero-vector (apply #'vector (make-list
;;                                        (length joint-names)
;;                                        :initial-element 0.0))))
;;     (roslisp:make-message
;;      "sensor_msgs/JointState"
;;      name joint-names-vector
;;      position joint-states
;;      velocity zero-vector
;;      effort zero-vector)))


;; (defun make-bullet-seed-states (robot-OBJECT robot-urdf joint-names &optional (steps 3))
;;   "Returns a sequence of possible seed states. The first seed state is
;; the current state represented by `robot' the other states are
;; generated by dividing the interval between the lower and the upper
;; limit of the joint into 1 + `steps' segments and using the corresponding
;; joint positions as seeds. `joint-names' is a list of strings"
;;   (declare (type robot-object robot-OBJECT) (type urdf robot-urdf))
;;   (flet ((init-limits (urdf lower upper joint-name)
;;            (let* ((joint (or (gethash joint-name (cl-urdf:joints urdf))
;;                              (error 'simple-error
;;                                     :format-control "Unknown joint `~a'"
;;                                     :format-arguments (list joint-name))))
;;                   (joint-limits (and (slot-boundp joint 'cl-urdf:limits)
;;                                      (cl-urdf:limits joint))))
;;              (case (cl-urdf:joint-type joint)
;;                ((:revolute :prismatic)
;;                   (setf (gethash joint-name lower)
;;                         (cl-urdf:lower joint-limits))
;;                   (setf (gethash joint-name upper)
;;                         (cl-urdf:upper joint-limits)))
;;                (:continuous
;;                   (setf (gethash joint-name lower)
;;                         (cl-urdf:lower joint-limits))
;;                   (setf (gethash joint-name upper)
;;                         (* pi 2)))
;;                (t (setf (gethash joint-name lower) 0.0)
;;                   (setf (gethash joint-name upper) 0.0))))))
;;     (let ((lower-limits (make-hash-table :test 'equal))
;;           (upper-limits (make-hash-table :test 'equal))
;;           (joint-names (map 'vector #'identity joint-names)))
;;       (map 'nil (curry #'init-limits robot-urdf lower-limits upper-limits) joint-names)
;;       (cons
;;        (btr:make-robot-joint-state-msg robot-OBJECT :joint-names joint-names)
;;        (cut:lazy-mapcar (lambda (joint-states)
;;                           (roslisp:make-msg
;;                            "sensor_msgs/JointState"
;;                            (stamp header) 0
;;                            name joint-names
;;                            position (reverse
;;                                      (map 'vector #'identity joint-states))
;;                            velocity (make-array (length joint-names)
;;                                                 :element-type 'float
;;                                                 :initial-element 0.0)
;;                            effort (make-array (length joint-names)
;;                                               :element-type 'float
;;                                               :initial-element 0.0)))
;;                         (apply #'lazy-cross-product
;;                                (reverse
;;                                 (loop for name across joint-names collecting
;;                                   (loop for i from 0 below steps collecting
;;                                     (+ (gethash name lower-limits)
;;                                        (* i (/ (- (gethash name upper-limits)
;;                                                   (gethash name lower-limits))
;;                                                (- steps 1)))))))))))))

(defun get-ee-pose-from-tcp-pose (tcp-pose
                                  &optional (tcp-in-ee-pose (cl-transforms:make-identity-pose)))
  (declare (type cl-transforms-stamped:pose-stamped tcp-pose)
           (type cl-transforms:pose tcp-in-ee-pose))
  (let ((goal-trans (cl-transforms:transform*
                     (cl-transforms:reference-transform tcp-pose)
                     (cl-transforms:transform-inv
                      (cl-transforms:reference-transform tcp-in-ee-pose)))))
    (cl-transforms-stamped:make-pose-stamped
     (cl-transforms-stamped:frame-id tcp-pose)
     (cl-transforms-stamped:stamp tcp-pose)
     (cl-transforms:translation goal-trans)
     (cl-transforms:rotation goal-trans))))


(defun call-kinematics-ik-service (cartesian-pose
                                   &key left-or-right seed-state
                                     (ik-base-frame cram-tf:*robot-torso-frame*)
                                     (tcp-in-ee-pose (cl-transforms:make-identity-pose)))
  (declare (type keyword left-or-right)
           (type cl-transforms-stamped:pose-stamped cartesian-pose))
  (let ((ik-link (get-pr2-ee-link left-or-right)))
    (handler-case
        (roslisp:with-fields ((error-code (val error_code))
                              (joint-state (joint_state solution)))
            (roslisp:call-persistent-service
             (get-ik-persistent-service left-or-right)
             (roslisp:make-request
              "moveit_msgs/GetPositionIK"
              (:ik_link_name :ik_request) ik-link
              (:pose_stamped :ik_request) (cl-transforms-stamped:to-msg
                                           (get-ee-pose-from-tcp-pose
                                            (cl-transforms-stamped:transform-pose-stamped
                                             cram-tf:*transformer*
                                             :pose (cl-transforms-stamped:copy-pose-stamped
                                                    cartesian-pose :stamp 0.0)
                                             :target-frame ik-base-frame
                                             :timeout cram-tf:*tf-default-timeout*)
                                            tcp-in-ee-pose))
              (:joint_state :robot_state :ik_request) (or seed-state
                                                          (make-zero-seed-state left-or-right))
              (:timeout :ik_request) 1.0))
          (if (eql error-code
                   (roslisp-msg-protocol:symbol-code
                    'moveit_msgs-msg:moveiterrorcodes
                    :success))
              joint-state
              (and (roslisp:ros-warn (call-kinematics-ik-service)
                                     "IK service failed: ~a"
                                     (roslisp-msg-protocol:code-symbol
                                      'moveit_msgs-msg:moveiterrorcodes
                                      error-code))
                   nil)))
      (roslisp:service-call-error ()
        (roslisp:ros-warn (call-kinematics-ik-service) "No IK solution found.")
        nil))))


(defun strip-joint-state (joint-state-msg left-or-right)
  "From the joint state message of a full robot extracts only the arm joints.
Uses SIDE EFFECTS!"
  (flet ((take-indexed-elems-from-list-into-vector (the-list indices)
           (map 'vector #'(lambda (a-position) (elt the-list a-position)) indices)))

   (let* ((joints (slot-value joint-state-msg 'sensor_msgs-msg:name))
          (indices (mapcar #'(lambda (item) (position item joints :test #'equal))
                           (get-pr2-arm-joints-list left-or-right))))
     (roslisp:with-fields (header name position)
         joint-state-msg
       (roslisp:make-message
        "sensor_msgs/JointState"
        :header header
        :name (take-indexed-elems-from-list-into-vector name indices)
        :position (take-indexed-elems-from-list-into-vector position indices))))))

(defun call-moveit-ik-service (cartesian-pose
                               &key left-or-right
                                 (ik-base-frame cram-tf:*robot-torso-frame*)
                                 (tcp-in-ee-pose (cl-transforms:make-identity-pose)))
  (declare (type cl-transforms-stamped:pose-stamped cartesian-pose)
           (type keyword left-or-right))
  (let* ((pose (cl-transforms-stamped:transform-pose-stamped
                cram-tf:*transformer*
                :pose (cl-transforms-stamped:copy-pose-stamped
                       cartesian-pose :stamp 0.0)
                :target-frame ik-base-frame
                :timeout cram-tf:*tf-default-timeout*)))
    (roslisp:with-fields ((solution (joint_state solution))
                          (error-code (val error_code)))
        (roslisp:call-persistent-service
         (get-ik-persistent-service :moveit)
         :ik_request
         (roslisp:make-msg
          "moveit_msgs/PositionIKRequest"
          ;; we assume that the last joint in JOINT-NAMES is the end
          ;; of the chain which is what we want for ik_link_name.
          ;; :ik_link_name (elt link-names 0)  <- moveit per default takes
          ;;                                      the last link in the chain
          :pose_stamped (cl-transforms-stamped:to-msg
                         (get-ee-pose-from-tcp-pose pose tcp-in-ee-pose))
          ;; something is wrong with the seed state atm, so this will stay
          ;; disabled for now
          ;; :robot_state (roslisp:make-msg
          ;;               "moveit_msgs/RobotState"
          ;;               :joint_state (or seed-state
          ;;                                (btr:make-robot-joint-state-msg robot)))
          :group_name (get-pr2-planning-group left-or-right)
          :timeout 1.0))
      (if (eql error-code (roslisp-msg-protocol:symbol-code
                           'moveit_msgs-msg:moveiterrorcodes
                           :success))
          (strip-joint-state solution left-or-right)
          (progn (roslisp:ros-warn (compute-iks) "IK moveit solver failed.")
                 nil)))))

(defmethod cram-robot-interfaces:compute-iks (pose-stamped
                                              &key link-name arm robot-state seed-state
                                                (pose-stamped-frame
                                                 cram-tf:*robot-torso-frame*)
                                                (tcp-in-ee-pose
                                                 (cl-transforms:make-identity-pose)))
  (declare (ignore link-name robot-state))
  (let ((solution
          (if *use-arm-kinematics-interface*
              (call-kinematics-ik-service pose-stamped
                                          :left-or-right arm
                                          :seed-state seed-state
                                          :ik-base-frame pose-stamped-frame
                                          :tcp-in-ee-pose tcp-in-ee-pose)
              (call-moveit-ik-service pose-stamped
                                      :left-or-right arm
                                      :ik-base-frame pose-stamped-frame
                                      :tcp-in-ee-pose tcp-in-ee-pose))))
    (when solution
      (list solution))))
