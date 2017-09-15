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

;;; This file defines some of the macros that make up the plan language, like seq, par, try-in-order, etc.

(defmacro def-plan-macro (name lambda-list &body body)
  "Wrapper around DEFMACRO for defining macros constituting cpl constructs
   used in plans.

   DEF-PLAN-MACRO wraps `body' in code that checks that it is only used within
   cpl plans. More precicesly the macro includes a check that it is used in a
   dynamic context where *CURRENT-TASK* is not nil. Otherwise an ERROR
   condition is signaled (at runtime).

   DEF-PLAN-MACRO also adds information about the plan macro (`lambda-list'
   and `body') to the symbol `name', so the plan walker can use this
   information during plan expansion. This information can also be used for
   plan transformation.

   See Also

     WALKER::EXPAND-PLAN"
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
  (let ((macro-name (or name "WITH-PARALLEL-CHILDS"))
        (child-name (or name "PARALLEL")))
    (with-unique-names (child-tasks child-states n-running n-done n-failed)
      `(with-task (:name ,macro-name)
         ,@(loop with n = (length child-forms)
                 for form in child-forms and i from 1
                 collect `(make-instance 'task
                            :name ',(format-gensym "[~A-CHILD-#~D/~D]-"
                                                   child-name i n)
                            :thread-fun (lambda () ,form)))
         (let* ((,child-tasks (child-tasks *current-task*))
                (,child-states (mapcar #'status ,child-tasks)))
           (wait-for (fl-apply #'notany (curry #'eq :created) ,child-states))
           (whenever ((apply #'fl-or (mapcar (rcurry #'fl-pulsed :handle-missed-pulses :once)
                                             ,child-states)))
             (multiple-value-bind (,n-running ,n-done ,n-failed)
                 (loop for task in ,child-tasks
                       when (task-running-p task) collect task into running
                       when (task-done-p task)    collect task into done
                       when (task-failed-p task)  collect task into failed
                       finally
                       (return (values running done failed)))
               (log-event
                 (:context "~A" ,macro-name)
                 (:display "~@[R: ~{~A~^, ~} ~]~
                            ~@[~:_D: ~{~A~^, ~} ~]~
                            ~@[~:_F: ~{~A~^, ~}~]"
                           (mapcar #'task-abbreviated-name ,n-running)
                           (mapcar #'task-abbreviated-name ,n-done)
                           (mapcar #'task-abbreviated-name ,n-failed))
                 (:tags ,(make-keyword macro-name)))
               ;; We have to take this detour (using N-FOO) because
               ;; WATCHER-BODY may legitimately contain IGNORE.
               (let ((,running ,n-running)
                     (,done    ,n-done)
                     (,failed  ,n-failed))
                 ,@watcher-body))))))))

;;; TODO@demmeln: Maybe put this top level stuff in a seperate file

;;; We could "pass" the task tree using the special variable *TASK-TREE*
;;; instead of a parameter to the setup-hook function, but the second way
;;; seems cleaner and doesn't expose this implementation detail (namlely
;;; *TASK-TREE*) to the user of the hook.
(define-hook cram-language::on-top-level-setup-hook (top-level-name task-tree)
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

(define-hook cram-language::on-top-level-cleanup-hook (top-level-name)
  (:documentation "All defined hooks are exected after the top level task has
  been joined. The call is made when the dynamic environment established by
  the return values of ON-TOP-LEVEL-SETUP-HOOK is still in effect. The call to
  this hook is protected by an unwind protect."))

;;; We need to manage the task trees somehow. The idea is to have a hash-table
;;; containing one task-tree for every top-level plan. Otherwise, the
;;; task-tree management would become really really messy.

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

;; TODO: Maybe introduce a register-top-level-task-tree akin to
;; register-top-level-episode-knowledge

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

(define-hook cram-language::on-preparing-named-top-level (name)
  (:documentation ""))

(define-hook cram-language::on-finishing-named-top-level (id)
  (:documentation ""))

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
           (clear-tasks *task-tree*) ; Note: this leaves the tree structure and
                                     ; code replacements intact.
           (let ((log-id (first (cram-language::on-preparing-named-top-level ',name))))
             (single-form-progv (cram-language::on-top-level-setup-hook ',name *task-tree*)
               (unwind-protect
                    (with-failure-handling
                        ((plan-failure (e)
                           (error e))
                         (error (e)
                           (error e)))
                      (let ((,task (make-instance 'toplevel-task
                                                  :name ',task-name
                                                  :thread-fun (lambda () ,@body))))
                        (unwind-protect-case ()
                                             (join-task ,task)
                          (:abort
                           ;; As TOP-LEVEL will be used from within a normal
                           ;; thread rather than task, we have to make sure that
                           ;; the task and all its children will get evaporated
                           ;; in case the thread is killed.
                           (evaporate ,task
                             :sync t
                             :reason ,(format nil "~A aborted." name))))))
                 (progn
                   (cram-language::on-finishing-named-top-level log-id)
                   (cram-language::on-top-level-cleanup-hook ',name)))))))))

(defmacro top-level (&body body)
  "Anonymous top-level, e.g. for interactive use. See NAMED-TOP-LEVEL for
   details."
  `(named-top-level () ,@body))

(def-plan-macro with-task ((&key (class 'task) (name "WITH-TASK")) &body body)
  "Executes body in a separate task and joins it."
  (let ((name (gensym (format nil "[~a]-" name))))
    (with-gensyms (task)
      `(let ((,task (make-instance ',class
                      :name ',name
                      :thread-fun (lambda () ,@body))))
         (join-task ,task)))))

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
            (assert-no-returning
              (signal (result (car failed-tasks)))))
           ((not running-tasks)
            (return (result (car (last finished-tasks))))))))

(defmacro :tag (name &body body)
  "Create a tag named name in the current lexical scope."
  (declare (ignore body))
  (error "(:TAG ~S ..) used outside of WITH-TAGS." name))

(define-hook cram-language::on-begin-execute-tag-task (name))
(define-hook cram-language::on-finish-execute-tag-task (id))

;;; - Don't nest with-tags calls, since code walking will mess up the result.
;;; - Don't use with-tags within macrolet/symbol-macrolet/..., if those macros expand
;;;   to something containig (:tag ...) forms. They won't be picked up by the code
;;;   walker and thus not
;;;   handled properly.
;;; - Don't use with-tags within flet/labels/let/... if those establish bindings that
;;;   shadow global macros that expand to something containing (:tag ...) forms, as
;;;   the code walker will assume the global macros and falsely pick up those tagged forms.
(def-plan-macro with-tags (&body body &environment lexenv)
  "Execute body with all tags bound to the corresponding lexically
   bound variables."
  (let ((tags (list)))
    (flet ((tags-handler (tag-name body)
             (declare (ignore body))
             (push tag-name tags)))
      (walk-with-tag-handler `(progn ,@body) #'tags-handler lexenv)
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
                        `(let* ((log-id (first (cram-language::on-begin-execute-tag-task ,name)))
                                (,',tag-path (cond ((executed ,name)
                                                    (let* ((last-path (task-path ,name))
                                                           (iter-path (cons (path-next-iteration (car last-path))
                                                                            (cdr last-path))))
                                                      (setf ,name (make-task :name ',name
                                                                             :path iter-path))
                                                      iter-path))
                                                   (t (cons `(tagged ,',name) ,',current-path)))))
                           (unwind-protect
                                (execute-task-tree-node
                                 (register-task-code ',tag-body (lambda () ,@tag-body)
                                                     :path ,',tag-path))
                             (cram-language::on-finish-execute-tag-task log-id)))))
             ,@body))))))

(def-plan-macro with-task-suspended ((task &key reason) &body body)
  "Execute body with 'task' being suspended."
  (with-gensyms (task-sym)
    `(let ((,task-sym ,task))
       (unwind-protect
            (progn
              (suspend ,task-sym :sync t :reason ,reason)
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
            (assert-no-returning
              (signal (result (car failed-tasks)))))
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
            (assert-no-returning
              (signal  (make-condition 'composite-failure
                                       :failures (mapcar #'result failed-tasks)))))
           (finished-tasks
            (return (result (car (last finished-tasks))))))))

(def-plan-macro try-in-order (&body forms)
  "Execute forms sequentially. Succeed if one succeeds, fail if all fail.
   In case of failure, a composite-failure is signaled."
  (with-gensyms (failures)
    `(block nil
       (let ((,failures (list)))
         ,@(mapcar (lambda (form)
                     ;; FIXME: TODO, use with-failure-handling
                     `(handler-case (return ,form)
                        (plan-failure (err)
                          (push err ,failures))))
                   forms)
         (assert-no-returning
           (signal
            (make-condition 'composite-failure
                            :failures (reverse ,failures))))))))

(def-plan-macro try-each-in-order ((variable list) &body body)
  "Executes `body' with `variable' bound to each element in `list'
  sequentially until `body' succeeds, i.e. returns the result of
  `body' as soon as `body' succeeds and stops iterating."
  (with-gensyms (failures)
    `(let ((,failures (list)))
       (dolist (,variable ,list (assert-no-returning
                                  (signal
                                   (make-condition 'composite-failure
                                                   :failures (reverse ,failures)))))
         (handler-case (return (progn ,@body))
           (plan-failure (condition)
             (push condition ,failures)))))))

;;; FIXME: circular ordering could be detected at compile-time.

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
         unless (eq sym :order)
           do (error "Malformed ordering constraint.")
            nconc `((,constraining-name ,constraining-task)
                    (,constrained-name ,constrained-task)) into bindings
            collect (list constraining-name constrained-name) into orders
            finally (return (values bindings orders)))
    `(let ,bindings
       ;;; This is not thread-unsafe because the tasks are supposed
       ;;; not to run yet. FIXME: this should probably be asserted.
       ,@(loop for (constraining constrained) in orders
               collect `(push (if (eql (cpl-impl::class-name
                                        (cpl-impl::class-of ,constraining))
                                       'cpl:value-fluent)
                                  (fl-pulsed ,constraining)
                                  (task-dead ,constraining))
                              (task-constraints ,constrained)))
       (par
         ,@steps))))

(def-plan-macro par-loop ((var sequence) &body body)
  "Executes body in parallel for each `var' in `sequence'."
  (alexandria:with-gensyms (loop-body evaluated-sequence)
    `(labels ((,loop-body (,evaluated-sequence)
                (cpl:par
                  (let ((,var (car ,evaluated-sequence)))
                    (cpl:seq ,@body))
                  (when (cdr ,evaluated-sequence)
                    (,loop-body (cdr ,evaluated-sequence))))))
       (,loop-body ,sequence))))
