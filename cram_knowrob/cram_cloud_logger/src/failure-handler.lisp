;;;
;;; Copyright (c) 2017-2022, Sebastian Koralewski <seba@cs.uni-bremen.de>
;;;
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

(in-package :ccl)

(defun init-failure-mapper ()
  (let ((failure-mapper (make-instance 'ccl::cram-2-knowrob-mapper))
    (definition
      '(("cram-common-failures:low-level-failure" "LowLevelFailure")
        ("cram-common-failures:actionlib-action-timed-out" "ActionlibTimeout")
        ("cram-common-failures:navigation-high-level-failure" "NavigationHighLevelFailure")
        ("cram-common-failures:navigation-goal-in-collision" "NavigationGoalInCollision")
        ("cram-common-failures:looking-high-level-failure" "LookingHighLevelFailure")
        ("cram-common-failures:object-unreachable" "ObjectUnreachable")
        ("cram-common-failures:manipulation-goal-in-collision" "ManipulationGoalInCollision")
        ("cram-common-failures:object-unfetchable" "ObjectUnfetchable")
        ("cram-common-failures:fetching-failed" "FetchingFailed")
        ("cram-common-failures:object-undeliverable" "ObjectUndeliverable")
        ("cram-common-failures:delivering-failed" "DeliveringFailed")
        ("cram-common-failures:object-nowhere-to-be-found" "ObjectNowhereToBeFound")
        ("cram-common-failures:searching-failed" "SearchingFailed")
        ("cram-common-failures:environment-manipulation-impossible" "EnvironmentManipulationImpossible")
        ("cram-common-failures:environment-unreachable" "EnvironmentUnreachable")
        ("cram-common-failures:manipulation-low-level-failure" "ManipulationLowLevelFailure")
        ("cram-common-failures:gripper-low-level-failure" "GripperLowLevelFailure")
        ("cram-common-failures:gripper-closed-completely" "GripperClosedCompletely")
        ("cram-common-failures:gripper-goal-not-reached" "GripperGoalNotReached")
        ("cram-common-failures:manipulation-goal-not-reached" "ManipulationGoalNotReached")
        ("cram-common-failures:manipulation-pose-unreachable" "ManipulationPoseUnreachable")
        ("cram-common-failures:navigation-low-level-failure" "NavigationLowLevelFailure")
        ("cram-common-failures:navigation-pose-unreachable" "NavigationPoseUnreachable")
        ("cram-common-failures:navigation-goal-not-reached" "NavigationGoalNotReached")
        ("cram-common-failures:perception-low-level-failure" "PerceptionLowLevelFailure")
        ("cram-common-failures:perception-object-not-found" "PerceptionObjectNotFound")
        ("cram-common-failures:perception-object-not-in-world" "PerceptionObjectNotInWorld")
        ("cram-common-failures:ptu-low-level-failure" "PTULowLevelFailure")
        ("cram-common-failures:ptu-goal-unreachable" "PTUGoalNotReached")
        ("cram-common-failures:ptu-goal-not-reached" "PTUGoalUnreachable")
        ("cram-common-failures:torso-low-level-failure" "TorsoLowLevelFailure")
        ("cram-common-failures:torso-goal-unreachable" "TorsoGoalUnreachable")
        ("cram-common-failures:torso-goal-not-reached" "TorsoGoalNotReached")
        ("cram-designators:designator-error" "DesignatorError")
        ("cram-executive:designator-reference-failure" "DesignatorReferenceFailure")
        ("cram-executive:designator-goal-parsing-failure" "DesignatorGoalParsingFailure"))))
    (ccl::add-definition-to-mapper definition failure-mapper)
    failure-mapper))

(defparameter *failure-mapper* (init-failure-mapper))

(defun get-failure-uri (cram-failure-name)
  (concatenate 'string "'http://www.ease-crc.org/ont/cram_failures.owl#CRAM" (get-failure-name cram-failure-name) "'"))

(defun get-failure-name (cram-failure-name)
  (let* ((lower-cram-failure-name (string-downcase cram-failure-name))
         (knowrob-failure-name (get-definition-from-mapper lower-cram-failure-name *failure-mapper*)))
    (when (not knowrob-failure-name) (setf knowrob-failure-name "Failure"))
    knowrob-failure-name))
