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

(defun kill-task-tree ()
  "Deletes the current task tree."
  (cpl-impl:remove-top-level-task-tree (get-top-level-name)))

(defun clean-task-tree ()
  "Purges replacements from all task tree nodes."
  (mapcar (lambda (task)
            (setf (cpl-impl::task-tree-node-code-replacements task) nil))
          (cpl:flatten-task-tree (gethash (get-top-level-name)
                                          cpl-impl:*top-level-task-trees*))))

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
   with highest priority or - if no prioritized rule is applicable -
   the most recently registered rule is applied."
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
            (apply-transformation-rule (cdr rule-to-apply))))
        (progn (roslisp:ros-info (plt)
                        "No rule applicable. Check for enabled/disabled rules.")
               nil))))

(defun apply-transformation-rule (lazy-bindings
                                  &optional (top-level-name (get-top-level-name)))
;; `lazy-bindings' must be of the following form:
;; (list-of-designators-and-paths list-of-paths-to-ignore)
;; where the first list contains pairs of designators and paths.
;; One such pair looks like this:
;; ((desig-a desig-b desig-c ..)
;;  path-to-inject-designators-into)
;; The designators replace the code in the given path and are executed sequentially.
;; Paths in the second list point to tasks, whose code will be ignored. 
;; Following is an example of the replacement in two tasks and some eliminated code.
;; ((((desig-a desig-b desig-c)
;;    path-1)
;;   ((desig-x desig-y)
;;    path-2))
;;  (path-a path-b path-x path-m path-n path-o))
  (roslisp:ros-info (plt)
                    "Applying BOTH-HANDS-TRANSPORTING-RULE to top-level-plan ~a."
                    top-level-name)
  (destructuring-bind
      ((key . desig-to-path)
       (other-key . obsolete-task-paths))
      (cut:lazy-car lazy-bindings)
    (declare (ignore key other-key))

    ;; execute all designators on their respective path
    ;; The LOOP macro causes weird behaviour in cpl-impl:replace-task-code
    (let ((index 0))
      (dolist (desig-path-pair desig-to-path)
        (cpl-impl:replace-task-code
         `(,(intern (format nil "BOTH-HANDS-TRANSFORM-~a" (incf index))))
         #'(lambda (&rest desig)
             (declare (ignore desig))
             (mapcar #'exe:perform (first desig-path-pair)))
         (second desig-path-pair) 
         (cpl-impl:get-top-level-task-tree top-level-name)))
      
      ;; ignore those obsolete tasks
      (flet ((ignore-desig (&rest desig)
               (declare (ignore desig))
               T))
        (loop for obsolete-path in obsolete-task-paths
              do (cpl-impl:replace-task-code
                  `(,(intern (format nil "BOTH-HANDS-TRANSFORM-OBSOLETE-~a" (incf index))))
                  #'ignore-desig
                  obsolete-path
                  (cpl-impl:get-top-level-task-tree top-level-name))))))
  (roslisp:ros-info (plt) "BOTH-HANDS-TRANSPORTING-RULE applied."))

(register-transformation-rule
 both-hands-to-target '(task-transporting-both-hands-to-target ?transform-data ?obsolete-paths))
