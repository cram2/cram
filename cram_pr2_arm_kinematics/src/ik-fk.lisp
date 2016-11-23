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


;;; only arm-kinematics, moveit is general, you give the chain, not ask for it

(defparameter *ik-info-service-names* '(:left "pr2_left_arm_kinematics/get_ik_solver_info"
                                        :right "pr2_right_arm_kinematics/get_ik_solver_info"))

(defparameter *fk-info-service-names* '(:left "pr2_left_arm_kinematics/get_fk_solver_info"
                                        :right "pr2_right_arm_kinematics/get_fk_solver_info"))

(defvar *ik-solver-infos* '(:left nil :right nil)
  "ik solver infos for the left and right arm ik solvers")

(defvar *fk-solver-infos* '(:left nil :right nil)
  "fk solver infos for the left and right arm fk solvers")

(defun get-ik-solver-info (left-or-right)
  "Only applicable to arm_kinematics package, with moveit the chain is given in the request."
  (or (getf *ik-solver-infos* left-or-right)
      (setf (getf *ik-solver-infos* left-or-right)
            (roslisp:with-fields (kinematic_solver_info)
                (roslisp:call-service
                 (getf *ik-info-service-names* left-or-right)
                 'moveit_msgs-srv:getkinematicsolverinfo)
              kinematic_solver_info))))

(defun get-ik-solver-joints (left-or-right)
  (roslisp:with-fields (joint_names)
      (get-ik-solver-info left-or-right)
    joint_names))

(defun get-ik-solver-link (left-or-right)
  (roslisp:with-fields (link_names)
      (get-ik-solver-info left-or-right)
    (aref link_names 0)))

(defun get-fk-solver-info (left-or-right)
  (or (getf *fk-solver-infos* left-or-right)
      (setf (getf *fk-solver-infos* left-or-right)
            (roslisp:with-fields (kinematic_solver_info)
                (roslisp:call-service
                 (getf *fk-info-service-names* left-or-right)
                 'moveit_msgs-srv:getkinematicsolverinfo)
              kinematic_solver_info))))

(defun get-fk-solver-joints (left-or-right)
  (roslisp:with-fields (joint_names)
      (get-fk-solver-info left-or-right)
    joint_names))

(defun get-fk-links (left-or-right)
  (ecase left-or-right
    (:left (vector "l_wrist_roll_link" "l_elbow_flex_link"))
    (:right (vector "r_wrist_roll_link" "r_elbow_flex_link"))))



(defun make-zero-seed-state (left-or-right)
  (let* ((joint-names (get-ik-solver-joints left-or-right))
         (zero-vector (apply #'vector (make-list
                                       (length joint-names)
                                       :initial-element 0.0))))
    (roslisp:make-message
     "sensor_msgs/JointState"
     name joint-names
     position zero-vector
     velocity zero-vector
     effort zero-vector)))

(defun make-current-seed-state (left-or-right)
  (let* ((joint-names-vector (get-ik-solver-joints left-or-right))
         (joint-names (map 'list #'identity joint-names-vector))
         (joint-states (map 'vector #'joint-state-position (joint-states joint-names)))
         (zero-vector (apply #'vector (make-list
                                       (length joint-names)
                                       :initial-element 0.0))))
    (roslisp:make-message
     "sensor_msgs/JointState"
     name joint-names-vector
     position joint-states
     velocity zero-vector
     effort zero-vector)))


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



;; add TCP frame option

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

(defun call-kinematics-ik-service (left-or-right cartesian-pose
                                   &key (ik-base-frame cram-tf:*robot-torso-frame*)
                                     (tcp-in-ee-pose (cl-transforms:make-identity-pose)))
  (declare (type keyword left-or-right)
           (type cl-transforms:pose cartesian-pose))
  (let ((ik-link (get-ik-solver-link left-or-right)))
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
                                            (cl-transforms-stamped:pose->pose-stamped
                                             ik-base-frame
                                             0.0
                                             cartesian-pose)
                                            tcp-in-ee-pose))
              (:joint_state :robot_state :ik_request) (make-zero-seed-state left-or-right)
              (:timeout :ik_request) 1.0))
          (cond ((eql error-code
                      (roslisp-msg-protocol:symbol-code
                       'moveit_msgs-msg:moveiterrorcodes
                       :success)) joint-state)
                ((eql error-code
                      (roslisp-msg-protocol:symbol-code
                       'moveit_msgs-msg:moveiterrorcodes
                       :no_ik_solution)) nil)
                (t (roslisp:ros-warn (call-kinematics-ik-service)
                                     "IK service failed: ~a"
                                     (roslisp-msg-protocol:code-symbol
                                      'moveit_msgs-msg:moveiterrorcodes
                                      error-code)))))
      (roslisp:service-call-error ()
        (roslisp:ros-warn (call-kinematics-ik-service) "No IK solution found.")
        nil))))

(defun strip-joint-state! (joint-state arm)
  "This hack is called 'oh I hate moveit' and will stay here until moveit
acquires a nicer ROS API which will never happen because Ioan got bought by
Google, or until somebody writes a nice ROS API wrapper around the moveit
C++ API which has a very low probability happening because ain't nobody's got
time for that :(..."
  (let* ((arm-joints `(("left_arm" "l_shoulder_pan_joint"
                                   "l_shoulder_lift_joint"
                                   "l_upper_arm_roll_joint"
                                   "l_elbow_flex_joint"
                                   "l_forearm_roll_joint"
                                   "l_wrist_flex_joint"
                                   "l_wrist_roll_joint")
                       ("right_arm"  "r_shoulder_pan_joint"
                                     "r_shoulder_lift_joint"
                                     "r_upper_arm_roll_joint"
                                     "r_elbow_flex_joint"
                                     "r_forearm_roll_joint"
                                     "r_wrist_flex_joint"
                                     "r_wrist_roll_joint")))
         (joints (slot-value joint-state 'sensor_msgs-msg::name))
         (indeces (mapcar #'(lambda (item) (position item joints :test #'equal))
                          (cdr (assoc arm arm-joints :test #'equal)))))
    (mapc #'(lambda (a-slot)
                (setf (slot-value joint-state a-slot)
                      (map 'vector #'(lambda (a-position)
                                       (elt (slot-value joint-state a-slot) a-position))
                           indeces)))
            `(sensor_msgs-msg::name sensor_msgs-msg::position))))

(defmethod call-moveit-ik-service (planning-group cartesian-pose
                                   &key (ik-base-frame cram-tf:*robot-torso-frame*)
                                     (tcp-in-ee-pose (cl-transforms:make-identity-pose)))
  (declare (type cl-transforms-stamped:pose-stamped cartesian-pose)
           (type string planning-group))
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
          :group_name planning-group
          :timeout 1.0))
      (when (eql error-code (roslisp-msg-protocol:symbol-code
                             'moveit_msgs-msg:moveiterrorcodes
                             :success))
        (strip-joint-state! solution planning-group)
        (list solution)))))

(defun call-fk-service (left-or-right link-names-vector
                        &optional (fk-base-frame "torso_lift_link"))
  (declare (type keyword left-or-right))
  (handler-case
      (roslisp:with-fields ((response-error-code (val error_code))
                            pose_stamped fk_link_names)
          (progn
            (roslisp:wait-for-service (concatenate 'string
                                                   (getf *ik-service-namespaces* left-or-right)
                                                   "/get_fk_solver_info") 10.0)
            (roslisp:call-service
             (concatenate 'string (getf *ik-service-namespaces* left-or-right) "/get_fk")
             "moveit_msgs/GetPositionFK"
             (roslisp:make-request
              "moveit_msgs/GetPositionFK"
              (:frame_id :header) fk-base-frame
              :fk_link_names (or link-names-vector (get-fk-links left-or-right))
              (:joint_state :robot_state) (make-current-seed-state left-or-right))))
        (cond ((eql response-error-code
                    (roslisp-msg-protocol:symbol-code
                     'moveit_msgs-msg:moveiterrorcodes
                     :success))
               (map 'list
                    (lambda (name pose-stamped-msg)
                            (list name (cl-transforms-stamped:from-msg pose-stamped-msg)))
                    fk_link_names
                    pose_stamped))
              (t (error 'simple-error
                        :format-control "FK service failed: ~a"
                        :format-arguments (list
                                           (roslisp-msg-protocol:code-symbol
                                            'moveit_msgs-msg:moveiterrorcodes
                                            response-error-code))))))
    (simple-error (e)
      (format t "~a~%FK service call freaked out. Hmmm...~%" e))))

(defun call-ik-service-and-joint-trajectory-action (left-or-right cartesian-pose-list)
  (call-joint-trajectory-action
   left-or-right
   (mapcar (lambda (cartesian-pose)
             (let ((ik-solution (call-ik-service left-or-right cartesian-pose)))
              (when ik-solution
                (roslisp:msg-slot-value ik-solution :position))))
           cartesian-pose-list)))






