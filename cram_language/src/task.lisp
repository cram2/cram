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

(defvar *current-task* nil
  "Dynamically bound current task.")

(defvar *task-pprint-verbosity* 0
  "Verbosity level for how TASK objects are printed.
   A value of 0 means that no information is printed that may depend
   on a lock.")

(defvar *current-path* nil
  "Contains the current path (in reverse order).")

(defvar *task-local-variables* nil)
(defmacro define-task-variable (name &optional (global-value nil gvp)
                                               (docstring nil docp)
                                     &key      (type nil typep))
  "Define `name' as a global and task-local variable with an initial value of
   `global-value', if given. Before the execution of a task, a new binding for
   `name' will be established within the task's thread. These thread local
   variables of the child thread are bound to the values they have in the
   parent thread at the time of creation of the child thread. In other word
   task propagate the current values of task-variables to their children, but
   in each thread the binding is thread local."
  `(progn
     (defvar ,name
       ,@(when gvp  (list global-value))
       ,@(when docp (list docstring)))
     (declaim (type ,(if typep type t) ,name))
     (eval-when (:load-toplevel :execute)
       (pushnew ',name *task-local-variables*))))

(define-task-variable *task-tree* nil)

(define-task-variable *current-task-tree-node* nil)

;;; Protocol definition of task
(defgeneric execute (task &key ignore-no-parent)
  (:documentation
   "When the task has been instantiated without a thread function or
    the function has not been executed yet, it can be executed with
    this method.  When `ignore-no-parent' is true, no error message is
    signaled when *CURRENT-TASK* is unbound."))

(defgeneric executed (task)
  (:documentation
   "Returns NIL if the task has not been executed yet."))

(defgeneric result (task)
  (:documentation
   "Returns the result of the thread. For multiple values, this
    function also returns multiple values."))

(deftype status-indicator ()
  "Cf. *STATUS-TRANSITIONS*"
  `(member :created
           :running 
           :suspended :waiting
           :succeeded :evaporated :failed))

(defgeneric status (task)
  (:documentation "Returns the status fluent of the task."))

(defgeneric child-tasks (task)
  (:documentation
   "Returns the (currently unsorted) list of child tasks."))

(defgeneric parent-task (task)
  (:documentation
   "Returns the parent task."))

(defgeneric register-child (task child)
  (:documentation
   "Registers a child task. It is evaporated when the parent task dies."))

(defgeneric terminate (task task-status &optional result-value)
  (:documentation
   "Terminates a task. The status' fluent and the result value are set
    and the task function is terminated.

    TERMINATE does nothing if the thread's status
    is :RUNNING, :SUSPENDED or :WAITING."))

(defgeneric suspend (task)
  (:documentation
   "Blocks a thread until it is terminated or awaken."))

(defgeneric wake-up (task)
  (:documentation
   "Wakes up `task' if it's suspended; otherwise do nothing."))

(defgeneric join-task (task)
  (:documentation
   "Waits for a task to finish. When it fails, this function rethrows
    the failure, when it succeeds, it returns the result of the
    task."))

(defgeneric task-path (task)
  (:documentation
   "Returns the path of the task."))

(defmacro with-status ((new-status &optional (task '*current-task*)) &body body)
  "Sets the task's status to `new-status' before executing body, and
   restores the old value at the end. The status is only restored to
   its original value if the thread status is still `new-status' upon
   executing the cleanup form."
  `(flet ((body-fun () ,@body))
     (if wait-status
         (call-with-saved-status #'body-fun ,task ,new-status)
         (body-fun))))

(eval-when (:compile-toplevel :load-toplevel :execute)
  (defparameter *status-transitions*
    '((:created    -> :running :evaporated)
      (:running    -> :suspended :waiting :succeeded :failed :evaporated)
      (:waiting    -> :running :suspended :failed :evaporated)
      (:suspended  -> :running :waiting :evaporated)
      (:succeeded  -> )
      (:failed     -> )
      (:evaporated -> ))
    "Valid transitions from one task's status to the next."))

(defun update-status (task new-status)
  (macrolet 
      ((check-status-transition (old new)
         `(ecase ,old 
            ,@(loop for (from -> . tos) in *status-transitions*
                collect `((,from)
                          (assert (member ,new ',tos)
                                  (,new)
                                  "Invalid status transition from ~S to ~S."
                                  ,old ,new))))))
    (let ((old-status (value (status task))))
      (check-status-transition old-status new-status)
      (setf (value (status task)) new-status))))

(defun call-with-saved-status (thunk task new-status)
  (check-type new-status status-indicator)
  (let ((old-status (and task (value (status task)))))
    (unwind-protect
         (progn
           (when task (update-status task new-status))
           (funcall thunk))
      (when (and task (eq (value (status task)) new-status))
        (update-status task old-status)))))

(defparameter +dead+
  '(:failed :succeeded :evaporated))

(defparameter +done+
  '(:succeeded :evaporated))

(defparameter +alive+
  '(:created :running :waiting :suspended))

(define-condition suspension (condition)
  ((suspend-handler :initform (required-argument :handler)
                    :initarg :handler
                    :reader suspend-handler))
  (:documentation
   "This condition is signaled _in the contour_ of a task that is
    supposed to be suspended. By establishing a handler for this
    condition and by means of the following restarts, a task can
    control if and when blocking occurs.

      IGNORE-REQUEST:
	the request for suspension should be ignored.

      SUSPEND:
	the suspension should proceed. (By explicitly invoking this
	restart, you can specify things that are supposed to happen
	before and after suspension.)

    If the condition is not handled, suspension proceeds implicitly.

    The suspension process is performed by invoking the function
    stored in the `suspension-handler' slot.

    You probably want to use one of the abstractions SUSPEND-PROTECT,
    ON-SUSPENSION, or WITHOUT-SUSPENSION."))

(defun signal-suspension (suspend-descr)
  ;; N.B. this implicitly associates the restarts with the implicitly
  ;; created condition. (Cf. "Notes" section of RESTART-CASE's
  ;; dictionary entry.)
  (restart-case (typecase suspend-descr
                  (suspension (signal suspend-descr))
                  (t (signal 'suspension :handler suspend-descr)))
    (ignore-request ()
      :report "Ignore SUSPENSION request."
      (return-from signal-suspension t))
    (suspend ()))			; fall through
  (funcall (typecase suspend-descr
             (suspension (suspend-handler suspend-descr))
             (t suspend-descr))))

(defmacro suspend-protect (form &body protection-forms)
  "When the current task is suspended during the execution of `form',
   execute `protection-forms' just before suspending."
  (with-gensyms (condition)
    `(handler-bind ((suspension (lambda (,condition)
                                  (declare (ignore ,condition))
                                  ;; Use unwind-protect here to prevent
                                  ;; abortion of suspension request by
                                  ;; non-local exit.
                                  (unwind-protect
                                       (progn ,@protection-forms)
                                    (invoke-restart 'suspend)))))
       ,form)))

(defun process-pending-suspensions ()
  (declare (special *pending-suspensions*))
  (loop for s in *pending-suspensions*
     do (signal-suspension s))
  (setf *pending-suspensions* nil))

(defmacro without-suspension (&body body)
  "Execution of `body' cannot be interrupted by a suspension. If a suspension "
  `(let ((*suspensions-disabled* t)
         (*pending-suspensions* nil))
     (declare (special *pending-suspensions* *suspensions-disabled*))
     (unwind-protect
          (handler-bind
              ((suspension (lambda (condition)
                             (when *suspensions-disabled*
                               (push condition *pending-suspensions*)
                               (ignore-request condition)))))
            ,@body)
       (process-pending-suspensions))))

(defmacro with-suspension (&body body)
  "Explicitly allows suspension. To be used within the dynamic context
   of WITHOUT-SUSPENSION."
  `(let ((*suspensions-disabled* nil))
     (declare (special *pending-suspensions* *suspensions-disabled*))
     (when *pending-suspensions*
       (process-pending-suspensions))
     ,@body))

(defmacro on-suspension (when-suspended-form &body body)
  "Executes `when-suspended-form' whenever a suspension signal occurs
   while `form' is being executed."
  `(handler-bind
       ((suspension (lambda  (condition)
                      (declare (ignore condition))
                      ,when-suspended-form)))
     ,@body))

(defmacro with-unsuspendable-lock (lock &body body)
  "Executes `body' with lock held and delays suspensions until
   execution of `body' finishes."
  `(without-suspension
     (with-lock-held (,lock)
       ,@body)))

(define-condition termination (condition)
  ((status :initform :evaporated :initarg :status :accessor status)
   (result :initform nil :initarg :result :accessor result))
  (:documentation
   "This condition is signaled _in the contour_ of a task that is
    supposed to be terminated. By establishing a handler for this
    condition and by means of the following restarts, a task can
    control if and when termnation is supposed to happen.

      IGNORE-REQUEST:
	the request for termination should be ignored.

    If the condition is not handled, termination proceeds implicitly.

    You probably want to use one of the abstractions
    WITHOUT-TERMINATION, ON-TERMINATION, or IGNORE-TERMINATION."))

(defun signal-termination (status &optional result)
  (let ((condition (cond ((typep status 'condition)
                          (assert (typep status 'termination))
                          status)
                         (t (make-condition 'termination
                                            :status status 
                                            :result result)))))
    (if (find-restart 'ignore-request condition)
        (signal condition)
        (restart-case (signal condition)      
          (ignore-request ()
            :report "Ignore TERMINATION request."
            t)))))

(defmacro with-termination-handler (handler &body body)
  "Executes `handler' whenever a termination signal occurs."
  (with-gensyms (handler-fun)
    `(let ((,handler-fun ,handler))
       (handler-bind
           ((termination (lambda (condition)
                           (funcall ,handler-fun
                                    (status condition)
                                    (result condition)))))
         ,@body))))

(defmacro ignore-termination (&body body)
  "Ignores all terminations occuring while `body' is running"
  `(handler-bind ((termination #'ignore-request))
     ,@body))

(defmacro without-termination (&body body)
  "Assures that `body' is finished before the termination request is
   processed."
  (with-gensyms (received-termination)
    `(let ((,received-termination nil))
       (unwind-protect
            (handler-bind
                ((termination (lambda (condition)
                                (setf ,received-termination condition)
                                (ignore-request condition))))
              ,@body)
         (when ,received-termination
           (signal-termination ,received-termination))))))

(defun ignore-request (&optional condition)
  "Invoke the IGNORE-REQUEST restart _associated with_ CONDITION."
  (invoke-restart (or (find-restart 'ignore-request condition)
                      (error "Bug: tried to invoke non-existing ~
                                   IGNORE-REQUEST restart."))))


