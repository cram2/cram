;;; Copyright (c) 2013, Georg Bartels <georg.bartels@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
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

(in-package :pr2-fccl-process-module)

(defvar *left-arm-controller-interface* nil "Holds a fccl-publisher-interface to talk to the left arm.")
(defvar *right-arm-controller-interface* nil "Holds a fccl-publisher-interface to talk to the right arm.")

(defun init-fccl-arm-controllers ()
  "Initializes the left and right arm fccl-publisher-interfaces for the PR2."
  (cram-fccl:ensure-fccl-initialized)
  (setf *left-arm-controller-interface*
        (cram-fccl:add-fccl-controller-interface
         "/left_arm_feature_controller/constraint_config"
         "/left_arm_feature_controller/constraint_command"
         "/left_arm_feature_controller/constraint_state"
         "pr2_left_arm_feature_controller"))
  (setf *right-arm-controller-interface*
        (cram-fccl:add-fccl-controller-interface
         "/right_arm_feature_controller/constraint_config"
         "/right_arm_feature_controller/constraint_command"
         "/right_arm_feature_controller/constraint_state"
         "pr2_right_arm_feature_controller")))

(cram-roslisp-common:register-ros-init-function init-fccl-arm-controllers)

(defun execute-several-motion-phases (motion-phases side)
  "Executes a series of 'movement-phases' on the 'side' arm. Every movement-phase is represented by a set of constraints."
  (declare (type list motion-phases)
           (type symbol side))
  (loop for phase in motion-phases
        do (execute-single-motion-phase phase side)))

(defun execute-single-motion-phase (constraints side)
  "Executes a single motion with end-goal 'constraints' on the 'side' arm. Blockingly waits on a fluent to signal the end of the execution."
  (declare (type list constraints)
           (symbol side))
  (let ((interface (ecase side
                     (:left *left-arm-controller-interface*)
                     (:right *right-arm-controller-interface*))))
    ;; start execution
    (cram-fccl:execute-constraints-motion constraints interface)
    ;; wait for fluent to signal finish
    (cram-language:wait-for 
     (cram-language:fl-funcall 
      #'fccl-controller-finished-p 
      (cram-fccl:get-constraints-state-fluent interface) 
      (cram-fccl:movement-id interface)))))