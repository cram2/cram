;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
2;;;
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

(in-package :cl-user)

(defpackage cram-common-failures
  (:nicknames #:common-fail)
  (:use #:cpl)
  (:export
   ;; common
   #:low-level-failure
   #:actionlib-action-timed-out
   #:high-level-failure
   ;; high-level
   #:navigation-high-level-failure
   #:navigation-goal-in-collision
   #:navigation-failure-pose-stamped
   #:looking-high-level-failure
   #:object-unreachable
   #:object-unreachable-object
   #:manipulation-goal-in-collision
   #:object-unfetchable
   #:object-unfetchable-object
   #:object-undeliverable
   #:object-undeliverable-object
   #:object-nowhere-to-be-found
   #:object-nowhere-to-be-found-object
   #:environment-manipulation-impossible
   #:environment-unreachable
   ;; manipulation
   #:manipulation-low-level-failure
   #:gripper-low-level-failure
   #:gripper-failure-action
   #:gripper-closed-completely
   #:gripper-goal-not-reached
   #:manipulation-goal-not-reached
   #:manipulation-pose-unreachable
   ;; navigation
   #:navigation-low-level-failure
   #:navigation-failure-location
   #:navigation-pose-unreachable
   #:navigation-goal-not-reached
   ;; perception
   #:perception-low-level-failure
   #:perception-object-not-found
   #:object-not-found-object
   ;; ptu
   #:ptu-low-level-failure
   #:ptu-goal-unreachable
   #:ptu-goal-not-reached
   ;; torso
   #:torso-low-level-failure
   #:torso-goal-unreachable
   #:torso-goal-not-reached))
