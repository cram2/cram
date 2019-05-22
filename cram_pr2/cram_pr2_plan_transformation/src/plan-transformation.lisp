;;;
;;; Copyright (c) 2019, Arthur Niedzwiecki <niedzwiecki@uni-bremen.de>
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

(in-package :plt)

(defparameter *top-level-name* :top-level)
(defun get-top-level-name ()
  *top-level-name*)

(defvar *transformation-rules* (make-hash-table :test 'eq))
(defvar *disabled-transformation-rules* '())
(defvar *rule-priority* '())

(defmacro register-transformation-rule (name predicate)
  `(setf (gethash ',name *transformation-rules*)
         ,predicate))

(defmacro disable-transformation-rule (name)
  `(pushnew ',name *disabled-transformation-rules*))

(defmacro enable-transformation-rule (name)
  `(setf *disabled-transformation-rules*
         (remove ',name *disabled-transformation-rules*)))

(defun prioritize-rule (superior-rule inferior-rule)
  (if (member superior-rule *rule-priority*)
      (when (or (not (member inferior-rule *rule-priority*))
                (and (member inferior-rule *rule-priority*)
                     (< (position inferior-rule *rule-priority*)
                        (position superior-rule *rule-priority*))))
        (setf *rule-priority* (remove inferior-rule *rule-priority*))
        (push inferior-rule (cdr (nthcdr (position superior-rule *rule-priority*) *rule-priority*))))
      (dolist (rule (list inferior-rule superior-rule)) (pushnew rule *rule-priority*))))

(defun apply-rules ()
  (let ((applicable-rules '())
        (raw-bindings))
    (loop for k being the hash-keys of *transformation-rules* do
      (unless (member k *disabled-transformation-rules*)
        (roslisp:ros-info (plt) "Checking predicate for rule ~a." k)
        (setf raw-bindings (prolog (gethash k *transformation-rules*)))
        (when raw-bindings
          (push `(,k . ,raw-bindings) applicable-rules))))
    (if applicable-rules
        (progn
          (let* ((most-important-rule
                   (car (remove nil (mapcar (alexandria:rcurry #'find (mapcar #'car applicable-rules))
                                            *rule-priority*))))
                 (rule-to-apply (if most-important-rule
                                    (find most-important-rule applicable-rules :key #'car)
                                    (car applicable-rules))))
            (funcall (car rule-to-apply) (cdr rule-to-apply))))
        (roslisp:ros-info (plt) "No rule applicable. Check the predicates and/or enable other rules."))))
      

