;;;
;;; Copyright (c) 2020, Arthur Niedzwiecki <niedzwiecki@uni-bremen.de>
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

(defvar *transformation-rules* (make-hash-table :test 'eq)
  "List of all registered transformation rules.")
(defvar *disabled-transformation-rules* '()
  "List of registered, but disabled transformation rules.")
(defvar *rule-priority* '()
  "Lists priority of rules as their names in ascending order.")

(defparameter *top-level-name* :top-level
  "The default task tree name.")

(defun get-top-level-name ()
  "Returns the default `*top-level-name*' if either 0 or more than one task tree
   exist, otherwise returns the only (current) existing task-tree's name."
  (let* ((task-trees (alexandria:hash-table-keys
                      cpl-impl:*top-level-task-trees*)))
    (if (equal 1 (length task-trees))
      (car task-trees)
      (progn (roslisp:ros-warn (plt)
                               "There are ~a task trees.Returning default: ~a."
                               (length task-trees) *top-level-name*)
             *top-level-name*))))

(defun reset-task-tree ()
  "Deletes the current task tree."
  (cpl-impl:remove-top-level-task-tree (get-top-level-name)))

(defmacro register-transformation-rule (name predicate)
  "Registers or updates a transformation rule.
   `name': The lisp name of a transformation function.
   `predicate': A sophisticated predicate that serves as applicability rule
                and input to the tranformation function."
  `(setf (gethash ',name *transformation-rules*)
         ,predicate))

(defmacro disable-transformation-rule (name)
  "Adds the rule's `name' to the list of disabled rules."
  `(pushnew ',name *disabled-transformation-rules*))

(defmacro enable-transformation-rule (name)
  "Opposite of `disable-transformation-rule',
   removes a rule from the disabled list."
  `(setf *disabled-transformation-rules*
         (remove ',name *disabled-transformation-rules*)))

(defun prioritize-rule (superior-rule inferior-rule)
  "Manages the `*rule-priority*' list such that `superior-rule's come
   before `inferior-rule's."
  (if (member superior-rule *rule-priority*)
      (when (or (not (member inferior-rule *rule-priority*))
                (and (member inferior-rule *rule-priority*)
                     (< (position inferior-rule *rule-priority*)
                        (position superior-rule *rule-priority*))))
        (setf *rule-priority* (remove inferior-rule *rule-priority*))
        (push inferior-rule
              (cdr (nthcdr (position superior-rule *rule-priority*)
                           *rule-priority*))))
      (dolist (rule (list inferior-rule superior-rule))
        (pushnew rule *rule-priority*))))

(defun apply-rules ()
  "Checks all registered, non-disabled rules for applicability, then applies one
   applicable transformation on the current task tree. The applicability is
   determined by whether the predicate given in `*transformation-rules*'
   resolves successful. If more than one rule is applicable, either the rule
   with highest priority or - if no priority is given - the most recently
   registered rule is applied."
  (let ((applicable-rules '())
        raw-bindings)
    (loop for k being the hash-keys of *transformation-rules* do
      (unless (member k *disabled-transformation-rules*)
        (roslisp:ros-info (plt) "Checking predicate for rule ~a." k)
        (setf raw-bindings (prolog (gethash k *transformation-rules*)))
        (when raw-bindings
          (push `(,k . ,raw-bindings) applicable-rules))))
    (if applicable-rules
        (progn
          (let* ((most-important-rule
                   (car (remove nil
                                (mapcar (alexandria:rcurry #'find
                                                           (mapcar #'car
                                                                   applicable-rules))
                                        *rule-priority*))))
                 (rule-to-apply (if most-important-rule
                                    (find most-important-rule
                                          applicable-rules
                                          :key
                                          #'car)
                                    (car applicable-rules))))
            (funcall (car rule-to-apply) (cdr rule-to-apply))))
        (progn (roslisp:ros-info (plt)
                        "No rule applicable. Check for enabled/disabled rules.")
               nil))))
      

