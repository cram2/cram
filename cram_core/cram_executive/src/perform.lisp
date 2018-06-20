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

(define-condition designator-reference-failure (cpl:simple-plan-failure desig:designator-error)
  ((result :initarg :result :reader result :initform nil))
  (:default-initargs :format-control "designator-failure"))
(define-condition designator-goal-parsing-failure (cpl:simple-plan-failure  desig:designator-error)
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

(cpl:declare-goal perform (designator)
  (declare (ignore designator))
  "Performs the action or motion defined by `designator'. This goal
  infers which process module to use and calls pm-execute in it.")

(cpl:def-goal (perform ?designator)
  (generic-perform ?designator))

(defgeneric generic-perform (designator)
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
                       but it isn't defined. Cannot perform action." designator command)))))
