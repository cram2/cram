;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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
;;;


(in-package :cpl-impl)

(defmacro def-plan-macro (name lambda-list &body body)
  (setf (get name 'plan-type) :plan-macro)
  (setf (get name 'plan-lambda-list) lambda-list)
  (setf (get name 'plan-sexp) body)
  (let* ((documentation (when (stringp (car body))
                          (car body)))
         (body (if documentation
                   (cdr body)
                   body)))
    `(defmacro ,name ,lambda-list
       ,documentation
       `(cond ((not *current-task*)
               (error (format nil "'~a' form not inside a plan." ',',name)))
              (t
               ,,@body)))))

(defmacro with-parallel-childs (name (running done failed) child-forms
                                &body watcher-body)
  "Execute `child-forms' in parallel and execute `watcher-body'
   whenever any child changes its status.

   Lexical bindings are established for `running', `done' and `failed'
   around `watcher-body', and bound to lists of all running, done and
   failed tasks. `watcher-body' is executed within an implicit block
   named NIL.

   `name' is supposed to be the name of the plan-macro that's
   implemented on top of WITH-PARALLEL-CHILDS. `name' will be used to
   name the tasks spawned for `child-forms'.

   All spawned child tasks will be terminated on leave."
  (with-unique-names (child-tasks child-states)
    `(with-task (:name ,(or name "WITH-PARALLEL-CHILDS"))
       ,@(loop with n = (length child-forms)
               for form in child-forms and i from 1
               collect `(make-instance 'task
                          :name ',(format-gensym "[~A-CHILD-#~D/~D]-"
                                                 (or name "PARALLEL") i n)
                          :thread-fun (lambda () ,form)))
       (let* ((,child-tasks (child-tasks *current-task*))
              (,child-states (mapcar #'status ,child-tasks)))
         (wait-for (fl-apply #'notany (curry #'eq :created) ,child-states))
         (block nil
           (whenever ((apply #'fl-or (mapcar (rcurry #'fl-pulsed :handle-missed-pulses :once)
                                             ,child-states)) :wait-status nil)
             (multiple-value-bind (,running ,done ,failed)
                 (loop for task in ,child-tasks
                       when (task-running-p task) collect task into running
                       when (task-done-p task)    collect task into done
                       when (task-failed-p task)  collect task into failed
                       finally
                         (return (values running done failed)))
               ,@watcher-body)))))))

;;; TODO@demmeln: Maybe put this top level stuff in a seperate file

;;; We could "pass" the task tree using the special variable *TASK-TREE*
;;; instead of a parameter to the setup-hook function, but the second way
;;; seems cleaner and doesn't expose this implementation detail (namlely
;;; *TASK-TREE*) to the user of the hook.
(define-hook on-top-level-setup-hook (top-level-name task-tree)
  (:documentation "All defined hooks are executed before the top level task is
  created. The task-tree passed is the root of the task-tree used in this top
  level. Each method must return a cons cell with a list of symbols in the car
  and a list of values in the cdr. The lists of all methods are concatenated
  and the passed to a progv to establish the dynamic environment where the
  top-level task is executed.")
  (:method-combination hooks
                       :hook-combination
                       (lambda (&rest results)
                         (loop for r in results
                            append (car r) into symbols
                            append (cdr r) into values
                            finally (return (cons symbols values))))))

(define-hook on-top-level-cleanup-hook (top-level-name)
  (:documentation "All defined hooks are exected after the top level task has
  been joined. The call is made when the dynamic environment established by
  the return values of ON-TOP-LEVEL-SETUP-HOOK is still in effect. The call to
  this hook is protected by an unwind protect."))

(defvar *top-level-task-trees* (make-hash-table :test 'eq)
  "The task tree for named top levels (top level plans) is stored in this
   hash-table by indexed by the plan name (which is a symbol).")

(defun remove-top-level-task-tree (name)
  "Removes the task tree for a named top level from the global hash table
   where those all named top level task trees. You usually won't need this,
   but you can use it to clean up after using named-top-level directly,
   e.g. in a test suite."
  (declare (type symbol name))
  (remhash name *top-level-task-trees*))

(defun get-top-level-task-tree (name)
  "Returns the task tree for a named top level (top level plan)."
  (declare (type symbol name))
  (or (gethash name *top-level-task-trees*)
      (setf (gethash name *top-level-task-trees*)
            (make-task-tree-node))))

(defmacro single-form-progv (symbols-and-values-cons &body body)
  "Like progv, but instead of getting the symbols and values from evaluating
   two forms, this version only takes a single form which should return a cons
   cell with the symbol list in the car and the value list in the cdr."
  (once-only (symbols-and-values-cons)
    `(progv (car ,symbols-and-values-cons) (cdr ,symbols-and-values-cons)
       ,@body)))

(defmacro named-top-level ((&key (name nil)) &body body)
  "Creates a new task, executes body in it and waits until it is finished. All
   plan macros can only be used within the dynamic scope of a top-level form.

   The `name' is used to save the task tree in *top-level-task-trees*."
  (declare (type symbol name))
  (with-unique-names (task)
    (let ((task-name (gensym (if name
                                 (format nil "[~a-TOP-LEVEL]-" name)
                                 "[TOP-LEVEL]-"))))
      `(let* ((*task-tree* (if ',name
                               (get-top-level-task-tree ',name)
                               (make-task-tree-node)))
              (*current-task-tree-node* *task-tree*)
              (*current-path* (list)))
         (declare (special *task-tree* *current-task-tree-node* *current-path*))
         (when *current-task*
           (error "top-level calls cannot be nested."))
         (clear-tasks *task-tree*) ;; TODO@demmeln: clearing seems not always to be a good idea...
         (single-form-progv (on-top-level-setup-hook ',name *task-tree*)
           (unwind-protect
                (let ((,task (make-instance 'task
                               :name ',task-name
                               :thread-fun (lambda () ,@body)
                               :ignore-no-parent t)))
                  (with-failure-handling
                      ((plan-error (e)
                         (error e))
                       (error (e)
                         (error e)))
                    (unwind-protect
                         (join-task ,task)
                      (terminate ,task :evaporated))))
             (on-top-level-cleanup-hook ',name)))))))

(defmacro top-level (&body body)
  "Annonnymous top-level, e.g. for interactive use. See NAMED-TOP-LEVEL for
   details."
  `(named-top-level () ,@body))

(def-plan-macro with-task ((&key (name "WITH-TASK")) &body body)
  "Executes body in a separate task and joins it."
  (with-gensyms (task)
    `(let ((,task (make-instance 'task
                    :name ',(gensym (format nil "[~a]-" name))
                    :thread-fun (lambda () ,@body))))
       (join-task ,task))))

(defmacro seq (&body forms)
  "Executes forms sequentially. Fails if one fail. Succeeds if all
   succeed."
  `(progn ,@forms))

(def-plan-macro par (&body forms)
  "Executes forms in parallel. Fails if one fails. Succeeds if all
   succeed."
  `(with-parallel-childs "PAR" (running-tasks finished-tasks failed-tasks)
       ,forms
     (cond (failed-tasks
            (fail (result (car failed-tasks))))
           ((not running-tasks)
            (return (result (car (last finished-tasks))))))))

(defmacro :tag (name &body body)
  "Create a tag named name in the current lexical scope."
  (declare (ignore body))
  (error (format nil ":tag '~a' used without a 'with-tags' environment." name)))

;;; - Don't nest with-tags calls, since code walking will mess up the result.
;;; - Don't use with-tags within macrole/symbol-macrolet/..., if those macros expand
;;;   to somethion containig (:tag ...) forms. They won't be picked up by the code
;;;   walker and thus not
;;;   handled properly.
;;; - Don't use with-tags withing flet/labels/let/... if those establish bindings that
;;;   shadow global macros that expand to something containing (:tag ...) forms, as
;;;   the code walker will assume the global macros and falsely pick up those tagged forms.
(def-plan-macro with-tags (&body body &environment lexenv)
  "Execute body with all tags bound to the corresponding lexically
   bound variables."
  (let ((tags (list))
        (tags-body `(progn ,@body)))
    (flet ((tags-handler (tag-name body)
             (declare (ignore body))
             (push tag-name tags)))
      (walk-with-tag-handler tags-body #'tags-handler lexenv)
      (with-gensyms (current-path tag-path)
        `(let* ((,current-path *current-path*)
                ,@(mapcar (lambda (tag)
                            `(,tag (make-task :name ',tag
                                              :path (cons `(tagged ,',tag) ,current-path))))
                          tags))
           (declare (ignorable ,current-path))
           (macrolet ((:tag (name &body tag-body)
                        ;; We need to handle loops. A tag can be
                        ;; executed several times. In those cases, we
                        ;; need to create a new task object and
                        ;; execute it.
                        `(let ((,',tag-path (cond ((executed ,name)
                                                   (let* ((last-path (task-path ,name))
                                                          (iter-path (cons (path-next-iteration (car last-path))
                                                                           (cdr last-path))))
                                                     (setf ,name (make-task :name ',name
                                                                            :path iter-path))
                                                     iter-path))
                                                  (t (cons `(tagged ,',name) ,',current-path)))))
                           (execute-task-tree-node
                            (register-task-code ',tag-body (lambda () ,@tag-body)
                                                :path ,',tag-path)))))
             (unwind-protect
                  ,tags-body
               ,@(mapcar (lambda (tag)
                           `(terminate ,tag :evaporated))
                         tags))))))))

(def-plan-macro with-task-suspended (task &body body)
  "Execute body with 'task' being suspended."
  (with-gensyms (task-sym)
    `(let ((,task-sym ,task))
       (unwind-protect
            (progn
              (suspend ,task-sym)
              (wait-for (fl-eq (status ,task-sym) :suspended))
              ,@body)
         (wake-up ,task-sym)))))

(def-plan-macro pursue (&body forms)
  "Execute forms in parallel. Succeed if one succeeds, fail if one
   fails."
  `(with-parallel-childs "PURSUE" (running-tasks finished-tasks failed-tasks)
       ,forms
     (declare (ignore running-tasks))
     (cond (failed-tasks
            (fail (result (car failed-tasks))))
           (finished-tasks
            (assert (eq (value (status (car finished-tasks))) :succeeded))
            (return (result (car finished-tasks)))))))

(def-plan-macro try-all (&body forms)
  "Try forms in parallel. Succeed if one succeeds, fail if all fail.
   In the case of a failure, a condition of type 'composite-failure'
   is signaled, containing the list of all error messages and data."
  `(with-parallel-childs "TRY-ALL" (running-tasks finished-tasks failed-tasks)
       ,forms
     (cond ((and (not running-tasks) (not finished-tasks) failed-tasks)
            (fail  (make-condition 'composite-failure
                                   :failures (mapcar #'result failed-tasks))))
           (finished-tasks
            (return (result (car (last finished-tasks))))))))

(def-plan-macro try-in-order (&body forms)
  "Execute forms sequentially. Succeed if one succeeds, fail if all fail.
   In case of failure, a composite-failure is signaled."
  (with-gensyms (failures)
    `(block nil
       (let ((,failures (list)))
         ,@(mapcar (lambda (form)
                     `(handler-case (return ,form)
                        (plan-error (err)
                          (push err ,failures))))
                   forms)
         (fail (make-condition 'composite-failure
                               :failures ,failures))))))

(def-plan-macro partial-order ((&body steps) &body orderings)
  "Specify ordering constraints for `steps'. `steps' are executed in
an implicit PAR form. `orderings' is a list of orderings. An ordering
always has the form:

  (:order <contstraining-task> <constrained-task>)

`constraining-task' and `constrained-task' are task objects. That
means, they can be either be defined in the current lexical
environment (over a :tag) or by either using the function TASK to
reference the task by its absolute path or the function SUB-TASK to
reference it by its path relative to the PARTIAL-ORDER form."
  (multiple-value-bind (bindings orders)
      (loop for (sym constraining-task constrained-task) in orderings
         for constraining-name = (gensym "CONSTRAINING-TASK-")
         for constrained-name = (gensym "CONSTRAINED-TASK-")
         unless (eq sym :order) do (error "Malformed ordering constraint.")
         nconc `((,constraining-name ,constraining-task)
                 (,constrained-name ,constrained-task)) into bindings
         collect (list constraining-name constrained-name) into orders
         finally (return (values bindings orders)))
    `(let ,bindings
       ,@(loop for (constraining constrained) in orders
            collect `(push (task-dead ,constraining)
                           (task-constraints ,constrained)))
       (par
         ,@steps))))
