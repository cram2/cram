;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(in-package :pr2-reachability-costmap)

(defvar *persistent-ik-services* (make-hash-table :test 'equal))

(defvar *ik-solver-info* (make-hash-table :test 'equal))

(defun get-persistent-service (name)
  (declare (type string name))
  (let ((service (gethash name *persistent-ik-services*)))
    (if (and service (roslisp:persistent-service-ok service))
        service
        (setf (gethash name *persistent-ik-services*)
              (make-instance 'roslisp:persistent-service
                :service-name name
                :service-type "moveit_msgs/GetPositionIK")))))

(defun get-ik-solver-info (namespace)
  (or (gethash namespace *ik-solver-info*)
      (setf (gethash namespace *ik-solver-info*)
            (roslisp:with-fields (kinematic_solver_info)
                (roslisp:call-service
                 (concatenate 'string namespace "/get_ik_solver_info")
                 "moveit_msgs/GetKinematicSolverInfo")
              kinematic_solver_info))))

(defun get-ik-solver-joints (namespace)
  (roslisp:with-fields (joint_names)
      (get-ik-solver-info namespace)
    joint_names))

(defun get-ik-solver-link (namespace)
  (roslisp:with-fields (link_names)
      (get-ik-solver-info namespace)
    (aref link_names 0)))

(defun make-seed-state (namespace)
  (let* ((joint-names (get-ik-solver-joints namespace))
         (zero-vector (apply #'vector (make-list
                                       (length joint-names)
                                       :initial-element 0.0))))
    (roslisp:make-msg
     "sensor_msgs/JointState"
     name joint-names
     position zero-vector
     velocity zero-vector
     effort zero-vector)))

(defun moveit-find-ik-solution (&key ik-link-name group-name ik-base-frame-name pose)
  (declare (type string ik-link-name)
           (type string group-name)
           (type string ik-base-frame-name)
           (type cl-transforms:pose pose))
  ;;(format t "IK for ~a in group ~a with base ~a~%" ik-link-name group-name ik-base-frame-name)
  (let* ((dummy nil))
    (declare (ignore dummy))
    (roslisp:with-fields ((error-code (val error_code))
                          (joint-state (joint_state solution)))
        (roslisp:call-service
          "/compute_ik" "moveit_msgs/GetPositionIK"
          :ik_request (roslisp:make-message "moveit_msgs/PositionIKRequest"
                        :ik_link_name ik-link-name
                        :group_name group-name
                        :avoid_collisions T
                        :pose_stamped (cl-transforms-stamped:to-msg
                                       (cl-transforms-stamped:pose->pose-stamped
                                        ik-base-frame-name 0.0 pose))
                        :timeout 0.005))
      (cond ((eql error-code
                  (roslisp-msg-protocol:symbol-code
                   'moveit_msgs-msg:moveiterrorcodes
                   :success))
             joint-state)
            ((eql error-code
                  (roslisp-msg-protocol:symbol-code
                   'moveit_msgs-msg:moveiterrorcodes
                   :no_ik_solution))
             nil)
            (t (error 'simple-error
                      :format-control "IK service failed: ~a"
                      :format-arguments (list
                                         (roslisp-msg-protocol:code-symbol
                                          'moveit_msgs-msg:moveiterrorcodes
                                          error-code))))))))

(defun find-ik-solution (&key service-namespace pose (ik-base-frame *robot-torso-frame*))
  (declare (type string service-namespace)
           (type cl-transforms:pose pose))
  (let ((ik-link (get-ik-solver-link service-namespace)))
    (roslisp:with-fields ((error-code (val error_code))
                          (joint-state (joint_state solution)))
        (roslisp:call-persistent-service
         (get-persistent-service (concatenate 'string service-namespace "/get_ik"))
         (roslisp:make-request
          "moveit_msgs/GetPositionIK"
          (:ik_link_name :ik_request) ik-link
          (:pose_stamped :ik_request) (cl-transforms-stamped:to-msg
                                       (cl-transforms-stamped:pose->pose-stamped
                                        ik-base-frame 0.0 pose))
          (:joint_state :ik_seed_state :ik_request) (make-seed-state service-namespace)
          (:timeout :ik_request) 1.0))
      (cond ((eql error-code
                  (roslisp-msg-protocol:symbol-code
                   'moveit_msgs-msg:moveiterrorcodes
                   :success))
             joint-state)
            ((eql error-code
                  (roslisp-msg-protocol:symbol-code
                   'moveit_msgs-msg:moveiterrorcodes
                   :no_ik_solution))
             nil)
            (t (error 'simple-error
                      :format-control "IK service failed: ~a"
                      :format-arguments (list
                                         (roslisp-msg-protocol:code-symbol
                                          'moveit_msgs-msg:moveiterrorcodes
                                          error-code))))))))
