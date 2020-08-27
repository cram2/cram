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

(in-package :donbot-pm)

;;;;;;;;;;;;;;;;;;;; GRIPPERS ;;;;;;;;;;;;;;;;;;;;;;;;

(cpm:def-process-module grippers-pm (motion-designator)
  (destructuring-bind (command action-type-or-position which-gripper
                       &optional effort-but-actually-slippage-param)
      (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:move-gripper-joint
       (donbot-ll:call-gripper-action
        :action-type-or-position action-type-or-position
        :left-or-right which-gripper
        :effort-but-actually-slippage-parameter effort-but-actually-slippage-param)))))

;;;;;;;;;;;;;;;;;;;; GISKARD ;;;;;;;;;;;;;;;;;;;;;;;;

(cpm:def-process-module giskard-pm (motion-designator)
  (destructuring-bind (command argument-1 &rest rest-args)
      (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:move-tcp
       (giskard:call-arm-cartesian-action
        :goal-pose-left argument-1
        :goal-pose-right (first rest-args)
        :collision-mode (second rest-args)
        :collision-object-b (third rest-args)
        :collision-object-b-link (fourth rest-args)
        :collision-object-a (fifth rest-args)
        :move-base (sixth rest-args)
        :prefer-base (seventh rest-args)
        :align-planes-left (eighth rest-args)
        :align-planes-right (ninth rest-args)))
      (cram-common-designators:move-joints
       (giskard:call-arm-joint-action
        :goal-configuration-left argument-1
        :goal-configuration-right (first rest-args)
        :align-planes-left (second rest-args)
        :align-planes-right (third rest-args)))
      (cram-common-designators:move-arm-pull
       (giskard:call-environment-manipulation-action
        :open-or-close :open
        :arm argument-1
        :handle-link (fifth rest-args)
        :joint-angle (second rest-args)
        :prefer-base (eighth rest-args)))
      (cram-common-designators:move-arm-push
       (giskard:call-environment-manipulation-action
        :open-or-close :close
        :arm argument-1
        :handle-link (fifth rest-args)
        :joint-angle (second rest-args)
        :prefer-base (eighth rest-args)))
      (cram-common-designators:move-head
       (when argument-1
         (giskard:call-neck-action
          :goal-pose argument-1))
       (when (car rest-args)
         (giskard:call-neck-joint-action
          :goal-configuration (car rest-args))))
      (cram-common-designators:move-base
       (giskard:call-base-action
        :goal-pose argument-1))
      (cram-common-designators:move-torso
       (giskard:call-torso-action
        :goal-joint-state argument-1)))))
