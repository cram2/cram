;;;
;;; Copyright (c) 2019, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(defsystem cram-donbot-low-level
  :author "Gayane Kazhoyan"
  :maintainer "Gayane Kazhoyan"
  :license "BSD"

  :depends-on (roslisp
               roslisp-msg-protocol     ; for ros-message class type
               roslisp-utilities
               actionlib
               cl-transforms-stamped
               cl-transforms
               cram-simple-actionlib-client
               cram-language ; for with-failure-handling to the least
               cram-utilities
               cram-prolog
               ;; cram-math ; for degrees->radians
               cram-tf
               cram-common-failures
               ;; for reading out arm joint names
               cram-robot-interfaces
               cram-joint-states
               cram-donbot-description
               ;; msgs for low-level communication
               slipping_control_msgs-msg
               slipping_control_msgs-srv
               sun_tactile_common-msg
               ;; geometry_msgs-msg ; for force-torque sensor wrench
               ;; std_srvs-srv ; for zeroing force-torque sensor
               ;; sensor_msgs-msg
               ;; visualization_msgs-msg
               ;; iai_wsg_50_msgs-msg
               ;; control_msgs-msg ; neck message
               ;; trajectory_msgs-msg ; also for neck
               ;; ;; iai_control_msgs-msg ; neck message
               ;; iai_dlr_msgs-msg
               )
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "grippers" :depends-on ("package"))))))
