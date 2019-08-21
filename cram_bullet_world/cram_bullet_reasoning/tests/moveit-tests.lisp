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

(in-package :btr-tests)

#+outdated-and-execution-error
(
(defun compute-ik-client (position-ik-request)
  (roslisp:with-ros-node ("moveit-test-client")
    (if (not (roslisp:wait-for-service "compute_ik" 10))
        (roslisp:ros-warn (bullet-reasoning-tests)
                          "Timed out waiting for service moveit/compute_ik")
        (roslisp:call-service "compute_ik"
                              "moveit_msgs/GetPositionIK"
                              :ik_request position-ik-request))))

(define-test test-moveit-service-setup
  (let* ((request (roslisp:make-message
                   "moveit_msgs/PositionIKRequest"
                   :pose_stamped (cl-transforms-stamped:to-msg
                                  (cl-transforms-stamped:make-pose-stamped
                                   "torso_lift_link"
                                   (roslisp:ros-time)
                                   (cl-transforms:make-3d-vector 0.45 0.2 0.31)
                                   (cl-transforms:make-identity-rotation)))
                   :group_name "right_arm"
                   :timeout 1.0))
         (result (compute-ik-client request)))
    (assert-eql (roslisp-msg-protocol:symbol-code 'moveit_msgs-msg:moveiterrorcodes
                                                  :success)
                (moveit_msgs-msg:val (moveit_msgs-srv:error_code result)))))
)
