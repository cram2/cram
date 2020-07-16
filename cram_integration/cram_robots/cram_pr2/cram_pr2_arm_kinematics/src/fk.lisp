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

(defparameter *fk-service-names* '(:moveit "moveit/compute_fk"
                                   :left "pr2_left_arm_kinematics/get_fk"
                                   :right "pr2_right_arm_kinematics/get_fk"))

(defvar *fk-solver-infos* '(:left nil :right nil)
  "fk solver infos for the left and right arm fk solvers")

(defparameter *fk-info-service-names* '(:left "pr2_left_arm_kinematics/get_fk_solver_info"
                                        :right "pr2_right_arm_kinematics/get_fk_solver_info"))

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

(defun call-fk-service (left-or-right link-names-vector
                        &optional (fk-base-frame "torso_lift_link"))
  (declare (type keyword left-or-right))
  (handler-case
      (roslisp:with-fields ((response-error-code (val error_code))
                            pose_stamped fk_link_names)
          (progn
            (roslisp:wait-for-service (getf *fk-info-service-names* left-or-right) 10.0)
            (roslisp:call-service
             (getf *fk-service-names* left-or-right)
             "moveit_msgs/GetPositionFK"
             (roslisp:make-request
              "moveit_msgs/GetPositionFK"
              (:frame_id :header) fk-base-frame
              :fk_link_names (or link-names-vector (get-fk-links left-or-right))
              (:joint_state :robot_state) (make-zero-seed-state left-or-right))))
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
