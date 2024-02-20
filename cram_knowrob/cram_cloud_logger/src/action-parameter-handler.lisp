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

(define-all-action-designator-parameter-logging-functions)

(defun log-action-designator-parameters-for-logged-action-designator (action-designator-parameters
                                                                      action-designator-logging-id)
  (format t "action-desig-parameters: ~a~%" action-designator-parameters)
  ;; hack for taking into account list of arms in desigs, e.g.(:gripper  (:left :right))
  (let* ((?temp-list))
    (map-into ?temp-list (lambda (action-designator-parameter)
                           (format t "desig parameter list: ~a~%" action-designator-parameters)
                           (let* ((parameter-name (first action-designator-parameter))
                                  (parameter-value ;;(second action-designator-parameter)
                                    ;;if value is a list
                                    (if (and (listp (second action-designator-parameter)) (equal parameter-name :GRIPPER))
                                        ;;true
                                        (progn (format t "--- the push ~a~%" (second action-designator-parameter))
                                               (push (list parameter-name (cadr (second action-designator-parameter))) action-designator-parameters)
                                               (format t "--- action-desig-param ~a~%" (car (second action-designator-parameter)))
                                               (car (second action-designator-parameter)))
                                        ;;false
                                        (progn
                                          (format t "second parameter: ~a~%" action-designator-parameter)
                                          (second action-designator-parameter)))))
              
                             (format t "~%parameter-name: ~a~%" parameter-name)
                             (format t "parameter-value: ~a~%" parameter-value)
                             (let* ((logging-function (get-logging-function-for-action-designator-parameter parameter-name))
                                    (action-type (get-knowrob-action-name (get-property-value-str action-designator-parameters :TYPE) ""))
                                    (logging-args (list action-designator-logging-id action-type parameter-value)))
                               (execute-logging-function-with-arguments
                                action-designator-logging-id action-type parameter-name logging-function logging-args))))
              action-designator-parameters)))

(defun execute-logging-function-with-arguments (action-designator-logging-id
                                                action-type
                                                parameter-name
                                                logging-parameter-function
                                                logging-parameter-function-arguments)
  (format t "[elf] action-desig-log-id: ~a~% action-type: ~a~% parameter-name ~a~% log-param-func: ~a~% log-param-func-args: ~a~%"
          action-designator-logging-id action-type parameter-name logging-parameter-function logging-parameter-function-arguments)
    (if logging-parameter-function 
        (apply logging-parameter-function logging-parameter-function-arguments)
        (let ((parameter-name-str (write-to-string parameter-name)))
          (when (string-not-equal ":TYPE" parameter-name-str)
            (send-comment action-designator-logging-id
                          (concatenate 'string "Unknown Parameter: " parameter-name-str " -####- "
                                       (write-to-string (cadr logging-parameter-function-arguments))))))))
