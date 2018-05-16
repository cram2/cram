;;;
;;; Copyright (c) 2017, Sebastian Koralewski <seba@cs.uni-bremen.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;       specific prior written permission
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

(cpl:define-task-variable *action-parents* '())
(defparameter *action-siblings* (make-hash-table))

(defmethod exe:generic-perform :around ((designator desig:action-designator))
  (if *is-logging-enabled*
      (let ((action-id (log-perform-call designator)) (is-parent-action nil))
        (cpl:with-failure-handling
            ((cpl:plan-failure (e)
               (log-cram-finish-action action-id)
               (send-task-success action-id "false")
               (log-failure action-id e)
               ;;(format t "failure string: ~a" (write-to-string e))
               (if is-parent-action
                   (send-batch-query))))
          (if cram-projection:*projection-environment*
            (send-performed-in-projection action-id "true")
            (send-performed-in-projection action-id "false"))
          (log-cram-sub-action (car *action-parents*) action-id)
          (log-cram-sibling-action (car *action-parents*) action-id)
          (if (not *action-parents*)
              (setq is-parent-action t))
          (push action-id *action-parents*)
          (let ((perform-result (call-next-method)))
            (log-cram-finish-action action-id)
            (when (and perform-result (typep perform-result 'desig:object-designator))
              (let ((name (desig:desig-prop-value perform-result :name)))
                (when name
                  (send-object-action-parameter action-id perform-result))))
            (send-task-success action-id "true")
            (if is-parent-action
                   (send-batch-query))
            perform-result)))
      (call-next-method)))

(defun log-perform-call (designator)
  (connect-to-cloud-logger)
  (if *is-client-connected*
      (let ((result "")
            (cram-action-name (get-designator-property-value-str designator :TYPE)))
        (setf result (get-value-of-json-prolog-dict
                      (cdaar
                       (send-cram-start-action
                        (get-knowrob-action-name cram-action-name)
                        " \\'TableSetting\\'"
                        (get-timestamp-for-logging)
                        "PV"
                        "ActionInst"))
                      "ActionInst"))
        (log-action-parameter designator result)
        result)
      "NOLOGGING"))

(defun log-failure (action-id failure-type)
  (let ((failure-str (write-to-string failure-type)))
    (send-rdf-query (convert-to-prolog-str action-id)
                    "knowrob:failure"
                    (convert-to-prolog-str (subseq failure-str 2 (search " " failure-str))))))

(defun log-cram-finish-action (action-id)
  (send-cram-finish-action
   (convert-to-prolog-str action-id ) (get-timestamp-for-logging)))

(defun log-cram-sub-action (parent-id child-id)
  (if parent-id 
      (progn
        (send-cram-set-subaction
         (convert-to-prolog-str parent-id)
         (convert-to-prolog-str child-id)))))

(defun log-cram-sibling-action (parent-id child-id)
  (let ((hash-value (gethash parent-id *action-siblings*)))
    (if hash-value
      (let ((previous-id (car (cpl:value hash-value))))
          (progn (log-cram-prev-action
                  child-id previous-id)
                 (log-cram-next-action
                  previous-id child-id)
                 (setf (cpl:value hash-value) (cons child-id (cpl:value hash-value)))
                 (setf  (gethash parent-id *action-siblings*) hash-value)))
      (setf (gethash parent-id *action-siblings*) (cpl:make-fluent :name parent-id :value (cons child-id '()))))))

(defun log-cram-prev-action (current-id previous-id)
 ( send-cram-previous-action (convert-to-prolog-str current-id) (convert-to-prolog-str previous-id)))

(defun log-cram-next-action (current-id next-id)
  (send-cram-next-action (convert-to-prolog-str current-id) (convert-to-prolog-str next-id)))
