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

(in-package :pr2-ll)

(defparameter *ik-service-namespaces* '(:left "pr2_left_arm_kinematics"
                                        :right "pr2_right_arm_kinematics"))

(defvar *ik-persistent-services* '(:left nil :right nil)
  "a persistent service for ik requests")

(defvar *ik-solver-infos* '(:left nil :right nil)
  "ik solver infos for the left and right arm ik solvers")

(defvar *fk-solver-infos* '(:left nil :right nil)
  "fk solver infos for the left and right arm fk solvers")

(defun get-ik-persistent-service (left-or-right)
  (declare (type keyword left-or-right))
  (let ((service (getf *ik-persistent-services* left-or-right)))
    (if (and service (roslisp:persistent-service-ok service))
        service
        (setf (getf *ik-persistent-services* left-or-right)
              (make-instance 'roslisp:persistent-service
                :service-name (concatenate 'string
                                           (getf *ik-service-namespaces* left-or-right)
                                           "/get_ik")
                :service-type "moveit_msgs/GetPositionIK")))))

(defun get-ik-solver-info (left-or-right)
  (or (getf *ik-solver-infos* left-or-right)
      (setf (getf *ik-solver-infos* left-or-right)
            (roslisp:with-fields (kinematic_solver_info)
                (roslisp:call-service
                 (concatenate 'string
                              (getf *ik-service-namespaces* left-or-right)
                              "/get_ik_solver_info")
                 "moveit_msgs/GetKinematicSolverInfo")
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
                 (concatenate 'string
                              (getf *ik-service-namespaces* left-or-right)
                              "/get_fk_solver_info")
                 "moveit_msgs/GetKinematicSolverInfo")
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

(defun call-ik-persistent-service (left-or-right cartesian-pose
                        &optional (ik-base-frame "torso_lift_link"))
  (declare (type keyword left-or-right)
           (type cl-transforms:pose cartesian-pose))
  (let ((ik-link (get-ik-solver-link left-or-right)))
    (roslisp:with-fields ((error-code error_code)
                          (joint-state (joint_state solution)))
        (roslisp:call-persistent-service
         (get-ik-persistent-service left-or-right)
         (roslisp:make-request
          "moveit_msgs/GetPositionIK"
          (:ik_link_name :ik_request) ik-link
          (:pose_stamped :ik_request) (cl-tf:to-msg
                                       (cl-tf:pose->pose-stamped ik-base-frame 0.0 cartesian-pose))
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
            (t (error 'simple-error
                      "IK service failed: ~a" 
                      (roslisp-msg-protocol:code-symbol
                       'moveit_msgs-msg:moveiterrorcodes
                       error-code)))))))

(defun call-ik-service (left-or-right cartesian-pose
                        &optional (ik-base-frame "torso_lift_link"))
  (declare (type keyword left-or-right)
           (type cl-transforms:pose cartesian-pose))
  (let ((ik-link (get-ik-solver-link left-or-right)))
    (handler-case
        (roslisp:with-fields ((response-error-code (val error_code))
                              (joint-state (joint_state solution)))
            (progn
              (roslisp:wait-for-service (concatenate 'string
                                                     (getf *ik-service-namespaces* left-or-right)
                                                     "/get_ik_solver_info") 10.0)
              (roslisp:call-service
               (concatenate 'string (getf *ik-service-namespaces* left-or-right) "/get_ik")
               "moveit_msgs/GetPositionIK"
               (roslisp:make-request
                "moveit_msgs/GetPositionIK"
                (:ik_link_name :ik_request) ik-link
                (:pose_stamped :ik_request) (cl-tf:to-msg
                                             (cl-tf:pose->pose-stamped ik-base-frame 0.0 cartesian-pose))
                (:joint_state :robot_state :ik_request) (make-current-seed-state left-or-right)
                (:timeout :ik_request) 1.0)))
          (cond ((eql response-error-code
                      (roslisp-msg-protocol:symbol-code
                       'moveit_msgs-msg:moveiterrorcodes
                       :success)) joint-state)
                ((eql response-error-code
                      (roslisp-msg-protocol:symbol-code
                       'moveit_msgs-msg:moveiterrorcodes
                       :no_ik_solution)) nil)
                (t (error 'simple-error
                          :format-control "IK service failed: ~a"
                          :format-arguments (list
                                             (roslisp-msg-protocol:code-symbol
                                              'moveit_msgs-msg:moveiterrorcodes
                                              response-error-code))))))
      (simple-error (e)
        (declare (ignore e))
        (format t "IK service call freaked out. No IK solution found.~%")))))

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
