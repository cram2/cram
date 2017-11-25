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

(in-package :exe)

(defvar *logged-action-list* nil)

(define-condition designator-reference-failure (cpl:simple-plan-failure)
  ((result :initarg :result :reader result :initform nil))
  (:default-initargs :format-control "designator-failure"))
(define-condition designator-goal-parsing-failure (cpl:simple-plan-failure)
  ((result :initarg :result :reader result :initform nil))
  (:default-initargs :format-control "designator-goal-parsing-failure"))

(defun try-reference-designator (designator &optional (error-message ""))
  (handler-case (reference designator)
    (desig:designator-error ()
      (cpl:fail 'designator-reference-failure
                :format-control "Designator ~a could not be resolved.~%~a"
                :format-arguments (list designator error-message)))))

(defun convert-desig-goal-to-occasion (keyword-expression)
  (handler-case (destructuring-bind (occasion &rest params)
                    keyword-expression
                  (cons ;; (intern (string-upcase occasion) :keyword)
                   occasion
                   params))
    (error (error-message)
      (cpl:fail 'designator-goal-parsing-failure
                :format-control "Designator goal ~a could not be parsed.~%~a"
                :format-arguments (list keyword-expression error-message)))))

(defun call-perform-with-logging (command arguments action-id)
  (cpl:with-failure-handling
      ((cpl:plan-failure (e)
         (log-cram-finish-action action-id)
         (ccl::send-task-success action-id "false")
         (format t "failure string: ~a" (write-to-string e))))
    (let ((perform-result
            (apply command arguments)))
      (log-cram-finish-action action-id)
      (ccl::send-task-success action-id "true")
      perform-result)))

(defgeneric perform (designator)
  (:documentation "If the action designator has a GOAL key it will be checked if the goal holds.
TODO: there might be multiple plans that can execute the same action designator.
In PMs the solution is: try-each-in-order.
In plans it would make more sense to explicitly specify the order.
For now we will only take the first action designator solution.
For future we can implement something like next-different-action-solution
similar to what we have for locations.")

  (:method ((designator motion-designator))
    (unless (cpm:matching-available-process-modules designator)
      (cpl:fail "No matching process module found for ~a" designator))
    (try-reference-designator designator "Cannot perform motion.")
    (cpm:pm-execute-matching designator))

  (:method ((designator action-designator))
    (if ccl::*is-logging-enabled*

        (let ((action-id (log-perform-call designator)))
          (destructuring-bind (command &rest arguments)
              (try-reference-designator designator)
            (if (fboundp command)
                (let ((desig-goal (desig-prop-value designator :goal)))
                  (if desig-goal
                      (let ((occasion (convert-desig-goal-to-occasion desig-goal)))
                        (if (cram-occasions-events:holds occasion)
                            (warn 'simple-warning
                                  :format-control "Action goal `~a' already achieved."
                                  :format-arguments (list occasion))
                            (call-perform-with-logging command arguments action-id))
                        (unless (cram-occasions-events:holds occasion)
                          (cpl:fail "Goal `~a' of action `~a' was not achieved."
                                    designator occasion)))
                      (call-perform-with-logging command arguments action-id)))
                (progn
                  (log-cram-finish-action action-id)
                  (ccl::send-task-success action-id "false")
                  (cpl:fail "Action designator `~a' resolved to cram function `~a', ~
                       but it isn't defined. Cannot perform action." designator command)))))

        (destructuring-bind (command &rest arguments)
            (try-reference-designator designator)
          (if (fboundp command)
              (let ((desig-goal (desig-prop-value designator :goal)))
                (if desig-goal
                    (let ((occasion (convert-desig-goal-to-occasion desig-goal)))
                      (if (cram-occasions-events:holds occasion)
                          (warn 'simple-warning
                                :format-control "Action goal `~a' already achieved."
                                :format-arguments (list occasion))
                          (apply command arguments))
                      (unless (cram-occasions-events:holds occasion)
                        (cpl:fail "Goal `~a' of action `~a' was not achieved."
                                  designator occasion)))
                    (apply command arguments)))
              (cpl:fail "Action designator `~a' resolved to cram function `~a', ~
                       but it isn't defined. Cannot perform action." designator command))))))

(defun log-perform-call (designator)
  (ccl::connect-to-cloud-logger)
  (if ccl::*is-client-connected*
      (let ((result "") (cram-action-name (get-designator-property-value-str designator :TYPE)))
        (setf result (ccl::get-value-of-json-prolog-dict (cdaar (ccl::send-cram-start-action (get-knowrob-action-name cram-action-name) " \\'DummyContext\\'" (get-timestamp-for-logging) "PV" "ActionInst")) "ActionInst"))
        (log-action-parameter designator result)
        result)
      "NOLOGGING"))

(defun get-knowrob-action-name (cram-action-name)
  (let ((knowrob-action-name cram-action-name))
    (cond ((string-equal cram-action-name "reaching")
           (setf knowrob-action-name "Reaching"))
          ((string-equal cram-action-name "retracting")
           (setf knowrob-action-name "Retracting"))
          ((string-equal cram-action-name "lifting")
           (setf knowrob-action-name "LiftingAGripper"))
          ((string-equal cram-action-name "putting")
           (setf knowrob-action-name "SinkingAGripper"))
          ((string-equal cram-action-name "setting-gripper")
           (setf knowrob-action-name "SettingAGripper"))
          ((string-equal cram-action-name "opening")
           (setf knowrob-action-name "OpeningAGripper"))
          ((string-equal cram-action-name "closing")
           (setf knowrob-action-name "ClosingAGripper"))
          ((string-equal cram-action-name "detecting")
           (setf knowrob-action-name "LookingForSomething"))
          ((string-equal cram-action-name "placing")
           (setf knowrob-action-name "PuttingDownAnObject"))
          ((string-equal cram-action-name "picking-up")
           (setf knowrob-action-name "PickingUpAnObject"))
          ((string-equal cram-action-name "releasing")
           (setf knowrob-action-name "ReleasingGraspOfSomething"))
          ((string-equal cram-action-name "gripping")
           (setf knowrob-action-name "AcquireGraspOfSomething"))
          ((string-equal cram-action-name "looking")
           (setf knowrob-action-name "LookingAtLocation"))
          ((string-equal cram-action-name "going")
           (setf knowrob-action-name "MovingToLocation")))
    (concatenate 'string "knowrob:" (ccl::convert-to-prolog-str knowrob-action-name))))

(defun get-timestamp-for-logging ()
  (write-to-string (truncate (cram-utilities:current-timestamp))))

(defun log-cram-finish-action(action-id)
  (ccl::send-cram-finish-action
   (ccl::convert-to-prolog-str action-id ) (get-timestamp-for-logging)))

(defun get-designator-property-value-str(designator property-keyname)
  (string (cadr(assoc property-keyname (properties designator)))))

(defun log-action-parameter (designator action-id)
  (cond ((desig-prop-value designator :effort) (ccl::send-effort-action-parameter action-id (write-to-string (desig-prop-value designator :effort))))
        ((desig-prop-value designator :object) (ccl::send-object-action-parameter action-id (desig-prop-value designator :object)))
        ((desig-prop-value designator :arm) (ccl::send-arm-action-parameter action-id (desig-prop-value designator :arm)))
        ((desig-prop-value designator :gripper) (ccl::send-gripper-action-parameter action-id (desig-prop-value designator :gripper)))
        ((desig-prop-value designator :left-poses) (ccl::send-pose-stamped-list-action-parameter action-id "leftPoses" (desig-prop-value designator :left-poses)))
        ((desig-prop-value designator :right-poses) (ccl::send-pose-stamped-list-action-parameter action-id "rightPoses" (desig-prop-value designator :right-poses)))
        ((desig-prop-value designator :location) (ccl::send-location-action-parameter action-id (desig-prop-value designator :location)))
        ((desig-prop-value designator :target) (ccl::send-target-action-parameter action-id (desig-prop-value designator :target)))))
