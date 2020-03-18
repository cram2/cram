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

(cpm:def-process-module giskard-pm (motion-designator)
  (destructuring-bind (command argument-1 &rest rest-arguments)
      (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:move-tcp
       (call-giskard-cartesian-action :goal-pose-left argument-1
                                      :goal-pose-right (first rest-arguments)
                                      :collision-mode (second rest-arguments)
                                      :collision-object-b (third rest-arguments)
                                      :collision-object-b-link (fourth rest-arguments)
                                      :collision-object-a (fifth rest-arguments)
                                      :move-the-ass (sixth rest-arguments)
                                      :constraints (seventh rest-arguments)
                                      ))
      (cram-common-designators:move-joints
       (call-giskard-joint-action :goal-configuration-left argument-1
                                  :goal-configuration-right (first rest-arguments)))
      (cram-common-designators:move-base
       (call-giskard-base-action :goal-pose argument-1))
      (cram-common-designators:move-torso
       (call-giskard-torso-action :goal-joint-state argument-1))
      (cram-common-designators:move-head
       (if (first rest-arguments)
         (call-giskard-neck-action :goal-configuration (first rest-arguments))
         (when argument-1
           (call-giskard-poi-neck-action :poi argument-1))))
      (cram-common-designators:move-gripper-joint
       (call-giskard-gripper-action :action-type-or-position argument-1
                                    :left-or-right (first rest-arguments)
                                    :effort (when (cdr rest-arguments) (second rest-arguments)))))))

(prolog:def-fact-group giskard-pm (cpm:matching-process-module
                                   cpm:available-process-module)

  (prolog:<- (cpm:matching-process-module ?motion-designator giskard-pm)
    (or (desig:desig-prop ?motion-designator (:type :moving-tcp))
        (desig:desig-prop ?motion-designator (:type :moving-arm-joints))
        (desig:desig-prop ?motion-designator (:type :going))
        (desig:desig-prop ?motion-designator (:type :moving-torso))
        (desig:desig-prop ?motion-designator (:type :looking))
        (desig:desig-prop ?motion-designator (:type :gripping))
        (desig:desig-prop ?motion-designator (:type :opening-gripper))
        (desig:desig-prop ?motion-designator (:type :closing-gripper))
        (desig:desig-prop ?motion-designator (:type :moving-gripper-joint))))

  (prolog:<- (cpm:available-process-module giskard-pm)
    (prolog:not (cpm:projection-running ?_))))

;;; The examples below are deprecated, so the designator changed, but the idea is the same
;;; Examples:
;;
;; (cram-process-modules:with-process-modules-running
;;     (giskard::giskard-pm)
;;   (cpl:top-level
;;     (cpm:pm-execute-matching
;;      (desig:a motion (type moving-tcp) (right-pose ((0.5 0.5 1.5) (0 0 0 1)))))))
;;
;; (cram-process-modules:with-process-modules-running
;;     (giskard::giskard-pm)
;;   (cpl:top-level
;;     (cpm:pm-execute-matching
;;      (desig:a motion
;;                (to move-arm)
;;                (right ((0.5 -0.5 1.5) (0 0 0 1)))
;;                (left ((0.5 0.5 1.5) (0 0 0 1)))))))
;;
;; (cram-process-modules:with-process-modules-running
;;     (giskard::giskard-pm)
;;   (cpl:top-level
;;     (cpm:pm-execute-matching
;;      (desig:a motion (to move-arm) (right (((1 1 1) (0 0 0 1)) nil ((1 1 1) (0 0 0 1))))))))
