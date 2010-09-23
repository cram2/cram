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

;;; These are task-local so the test suite can use them to get all the
;;; tasks spawned during the extent of a test run.

(define-task-variable *save-tasks* nil
  "When t, every task to be executed is pushed to *tasks*.")

(define-task-variable *tasks* (make-queue)
  "Queue of all running tasks."
  :type queue)

(defun list-saved-tasks ()
  (list-queue-contents *tasks*))

(defun clear-saved-tasks ()
  (setf *tasks* (make-queue)))

(defclass task ()
  ((name
    :reader task-name
    :initarg :name
    :initform (gensym "UNKNOWN-")
    :type (or symbol string null)
    :documentation "The name of the task. Mostly for debugging reasons.
                    Should also become the name of the thread executing
                    the task.")
   (thread
    :initform nil
    :type (or thread null)
    :documentation "The internal thread object of the lisp implementation.")
   (parent-task
    :initform nil
    :reader parent-task
    :type (or task null)
    :documentation "The parent task.")
   (child-tasks
    :initform (list)
    :accessor child-tasks
    :type (list-of task) 
    :documentation "The list of child tasks.")
   (status                              ; initialized below
    :reader status
    :type fluent
    :documentation "Fluent indicating the task status. 
                    Cf. STATUS-INDICATOR.")
   (unblocked
    :initform (make-fluent :name (gensym "TASK-UNBLOCKED-") :value t)
    :documentation "For internal use. nil when the thread is blocked.")
   (result
    :initform nil
    :type (or list (member nil) condition)
    :documentation "The result of the task. When it terminates
		    normally, the slot contains the list of return
		    values. For evaporated tasks, it contains nil and
		    for failed tasks, it contains the condition
		    object.")
   (thread-fun
    :initform nil
    :initarg :thread-fun
    ;; FIXME: Why may this be NIL? Add explanation to docstring.
    :type (or function null)
    :documentation "The function of the task.")
   (path
    :initform *current-path*
    :initarg :path
    :reader task-path
    :type list
    :documentation "The path of the task.")
   (code
    :initform (and *current-task-tree-node*
                   (task-tree-node-code *current-task-tree-node*))
    :reader task-code
    :type (or code null)
    :documentation "The code of the path.")
   (constraints
    :initform (list)
    :accessor task-constraints
    :type (list-of fluent)
    :documentation "The list of ordering constraint fluents. The task
		    is not allowed to start running before all fluents
		    are true.")))

(defmethod initialize-instance :after
    ((task task) &key (ignore-no-parent nil)
                      (run-thread t))
  (with-slots (name status thread-fun parent-task child-tasks) task
    (setf status (make-fluent :name (format-gensym "~A-STATUS" name)
                              :value :created))
    (when (and thread-fun run-thread)
      (execute task :ignore-no-parent ignore-no-parent))))

(defmethod print-object ((task task) stream)
  (let ((level *task-pprint-verbosity*))
    (print-unreadable-object (task stream :type t :identity t)
      (format stream "~S" (string (task-name task)))
      ;; We depend on the verbosity level here because VALUE goes over
      ;; a lock.
      (when (> level 0)
        (format stream " ~S" (value (status task)))))))

(defun task-alive (task)
  "Returns a fluent indicating if the task is alive"
  (fl-funcall #'member (status task) +alive+))

(defun task-dead (task)
  (fl-funcall #'member (status task) +dead+))

(defun task-running-p (task)
  (member (value (status task)) +alive+))

(defun task-dead-p (task)
  (member (value (status task)) +dead+))

(defun task-done-p (task)
  (member (value (status task)) +done+))

(defun task-failed-p (task)
  (eq (value (status task)) :failed))

(defun task-body (task)
  (unwind-protect
       (with-slots (status parent-task constraints thread-fun) task
         (let ((*current-task* task))
           (declare (special *current-task*))
           (flet ((task-error-handler (condition)
                    (restart-case
                        (invoke-debugger condition)
                      (terminate-task ()
                        :report "Terminate failed task."
                        (terminate task :failed
                                   (make-condition 'rethrown-error
                                                   :error condition))))))
             (handler-bind
                 ((rethrown-error
                   (lambda (condition)
                     (terminate task :failed condition)))
                  (plan-error
                   (lambda (condition)
                     (if *break-on-plan-failures*
                         (task-error-handler condition)
                         (terminate task :failed
                                    (make-condition 'rethrown-error
                                                    :error condition)))))
                  (error (lambda (condition)
                           (if *break-on-errors*
                               (task-error-handler condition)
                               (terminate task :failed
                                          (make-condition 'rethrown-error
                                                          :error condition))))))
               (update-status task :running)
               (when *save-tasks*
                 (enqueue task *tasks*))
               (wait-for (apply #'fl-and constraints))
               (let ((result (multiple-value-list (funcall thread-fun))))
                 (terminate task :succeeded result))))))
    (loop for child in (child-tasks task) do
          (terminate child :evaporated))))

(defun task-terminate (task status result)
  (unless (task-dead-p task)
    (setf (slot-value task 'result) result
          (value (status task)) status)))

(defmethod execute ((task task) &key (ignore-no-parent nil)) 
  (assert (not (null (slot-value task 'thread-fun))))
  (without-termination
    (let* ((lock (make-lock))
           (thread-barrier (make-condition-variable :lock lock))
           (barrier-pulsed nil))
      ;; Register the task in the corresponding node.
      (flet ((doit ()
               (block nil
                 (flet ((do-terminate (status result)
                          (ignore-termination
                            (task-terminate task status result)
                            (return result))))
                   (with-termination-handler #'do-terminate
                     (unwind-protect
                          (progn
                            (with-lock-held (lock)
                              (unless barrier-pulsed
                                (condition-variable-wait thread-barrier)))
                            (unless (task-dead-p task)
                              (task-body task)
                              (warn "task body terminated.")))
                       (unless (task-dead-p task)
                         (warn "Unhandled exit ~a ~a" task (status task)))))))))
        (unless (or *current-task* ignore-no-parent)
          (error "No current task active (*CURRENT-TASK* is NIL)."))
        (with-slots (thread result parent-task) task
          (when thread
            (error "Task already runs a thread."))
          ;; The barrier assures that the thread slot is bound when
          ;; the task-body runs.
          (unless (task-dead-p task)
            (when *current-task*
              (register-child *current-task* task))
            (setf parent-task *current-task*)
            (unwind-protect
                 (setf thread (spawn-thread-for-task task #'doit))
              (with-lock-held (lock)
                (setf barrier-pulsed t)
                (condition-variable-signal thread-barrier)))
            (wait-for (fl-not (fl-eq (status task) :created)))))))))

(defun spawn-thread-for-task (task thunk)
  "Like SPAWN-THREAD but establishes task-specific context in the
   spawned thread, like for instance the task-local variables."
  (multiple-value-bind (bound-tlvs unbound-tlvs)
      (partition #'boundp *task-local-variables*)
    (spawn-thread (slot-value task 'name)
                  (let ((task-local-values (mapcar #'symbol-value bound-tlvs)))
                    #'(lambda ()
                        (progv bound-tlvs task-local-values
                          (progv unbound-tlvs nil ; DTRT.
                            (let ((*current-task* task)
                                  (*current-path* (task-path task)))
                              (funcall thunk)))))))))

(defmethod terminate ((task task) task-status &optional (result-value nil))
  (if (eq *current-task* task)
      (signal-termination task-status result-value)
      (without-termination
        (with-slots (thread status) task
          (cond (thread
                 ;; We want to ignore termination errors. They may
                 ;; happen when the thread already died. Checking for
                 ;; a running thread does not help because it can die
                 ;; between the check and termination.
                 (ignore-errors
                   (run-in-thread thread
                    (lambda ()
                      (unless (signal-termination task-status result-value)
                        ;; something bad has happened. Our signal
                        ;; handler is not yet set up.
                        (task-terminate task task-status result-value))))))
                (t
                 (unless (task-dead-p task)
                   (update-status task task-status))))))))

(defmethod executed ((task task))
  (not (null (slot-value task 'thread))))

(defmethod result ((task task))
  (with-slots (result) task
    (if (listp result)
        (apply #'values result)
        result)))

(defmethod register-child ((task task) child)
  ;; If, at some point in the future, we want to lax this restriction, we must
  ;; make sure that this operation is properly synchronized (either here, or
  ;; decide that the caller is responsible for synchronization.)
  ;;
  ;; NOTE: #demmeln: How does this ensure thread safety? It ensures that each
  ;; task may only register childs to itself, but what if another thread
  ;; traverses the list of children of this thread, while it is modified?
  ;; (like SUSPEND does)?
  (assert (eq task *current-task*))
  (when (member child (slot-value task 'child-tasks))
    (warn "Registring allready registerd child ~s in task ~s" child task))
  (pushnew child (slot-value task 'child-tasks))
  task)

(defmethod suspend ((task task))
  (with-slots (thread unblocked status) task
    (flet ((suspend-function ()
             ;; (wait-for unblocked) results in spurious wake-ups due
             ;; to pulses. We need to use a specific fluent net to
             ;; really wait for a truth value here.
             (wait-for (fl-not (fl-eq unblocked nil))
                       :wait-status :suspended)))
      (wait-for (fl-not (fl-eq status :created)))
      (loop for child in (child-tasks task) do (suspend child))
      (ignore-some-conditions (sb-thread:interrupt-thread-error)
        (run-in-thread thread
                       (lambda ()
                         (when (member (value (status task)) '(:running :waiting))
                           (setf (value unblocked) nil)
                           (signal-suspension #'suspend-function))))))))

(defmethod wake-up ((task task))
  (when (value (task-alive task))
    (setf (value (slot-value task 'unblocked)) t)
    (loop for child in (child-tasks task) do (wake-up child))))

(defmethod join-task ((task task))
  (when (eq task *current-task*)
    (error "Cannot join current task."))
  (wait-for (task-dead task))
  (if (task-failed-p task)
      (fail (result task))
      (result task)))
