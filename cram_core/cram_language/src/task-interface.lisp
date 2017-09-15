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

(defun current-task ()
  (or *current-task* (make-dummy-task (current-thread))))

(defvar *task-pprint-verbosity* 0
  "Verbosity level for how TASK objects are printed.
   A value of 0 means that no information is printed that may depend
   on a lock.")

(defvar *current-path* nil
  "Contains the current path (in reverse order).")

(defvar *task-local-variables* nil)
(defmacro define-task-variable (name &optional (global-value nil gvp)
                                               (docstring nil docp)
                                     &key (type nil typep)
                                          (init-function nil)
                                          (ignore-package-locks))
  "Define a binding for `name' as a special variable, and in
particular as a task variable.

`global-value' is the variable's global value; if not given, the
variable is unbound. `docstring' is its documentation string. `type'
is, optionally, its globally proclaimed type.

A task variable is a binding that is established and initialized right
before execution of a task. The binding is task-local, meaning that
assignment to it won't affect the variable's global binding, or
bindings within other tasks.

The initialization is performed by `init-function' which must be a
function of three parameters: the TASK object going to be executed,
the TASK object of the parent task, and the previous value of `name'
in the parent. In case the task object is a TOPLEVEL-TASK, the second
argument will be NIL, and the third one will be the global value.

Task variables can hence be used for different purposes:

     1. to initialize a variable based on the task object to be
        executed.

     2. to initialize a variable based on the task's parent.

     3. to inherit a task variable's value from its parent.

In case `init-function' is not given, the default behaviour is the
behaviour of the 3rd item."
  `(progn
     ;; Kludge: SBCL signals package-lock-violation for (DECLAIM
     ;; (SPECIAL)) despite SB-EXT:DISABLE-PACKAGE-LOCKS.
     ,(if ignore-package-locks
          `(setq ,name ,global-value)
          `(defvar ,name
             ,@(when gvp  (list global-value))
             ,@(when docp (list docstring))))
     ,(when typep
       `(declaim (type ,type ,name)))
     (eval-when (:load-toplevel :execute)
       (setf (assoc-value *task-local-variables* ',name)
             ,(or init-function '#'inherit-parent-value)))))

(defun inherit-parent-value (task parent-task parent-value)
  (declare (ignore task parent-task))
  parent-value)

;;; Forward declarations

(defclass abstract-task ()
  ())

(defclass task (abstract-task)
  ())

(defclass toplevel-task (abstract-task)
  ())

;;; Protocol definition of task

(defgeneric execute (task)
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

(defgeneric status (task)
  (:documentation "Returns the status fluent of the task."))

(deftype status-indicator ()
  "Cf. *STATUS-TRANSITIONS*"
  `(member :created
           :running 
           :suspended
           :succeeded :evaporated :failed))

(defparameter +dead+
  '(:failed :succeeded :evaporated))

(defparameter +done+
  '(:succeeded :evaporated))

(defparameter +alive+
  '(:created :running :waiting :suspended))

(defgeneric child-tasks (task)
  (:documentation
   "Returns the (currently unsorted) list of child tasks."))

(defgeneric parent-task (task)
  (:documentation
   "Returns the parent task."))

(defgeneric evaporate (task &key reason sync)
  (:documentation
   "Terminates a task."))

(defgeneric suspend (task &key reason sync)
  (:documentation
   "Blocks a thread until it is terminated or awaken."))

(defgeneric wake-up (task &key reason sync)
  (:documentation
   "Wakes up `task' if it's suspended; otherwise do nothing."))

(defgeneric join-task (task)
  (:documentation
   "Waits for `task' to finish.
    When `task' fails, this function rethrows the failure; when `task'
    succeeds, it returns the result values of `task'."))

(defgeneric task-path (task)
  (:documentation
   "Returns the path of the task."))


;;; Macros

(defmacro without-scheduling (&body body)
  "Execute `body' without periodically entering into a task's event
   loop. This prevents all acting on messages (including suspension,
   evaporation, etc.) from other tasks."
  `(call-without-scheduling #'(lambda () ,@body)))

(defmacro with-scheduling (&body body)
  "Enable scheduling again."
  `(call-with-scheduling #'(lambda () ,@body)))

(defmacro retry-after-suspension (&body body)
  "Execute `body', and return its values.

   In case a suspension event occurs during the execution, `body' is
   completely unwound, and the suspension takes place _outside_ of
   `body'. After wakeup, `body' is executed again.

   The intended use case is to unwind out of critical sections to
   ensure that a task won't suspend while holding on locks. "
  (with-unique-names (retry retry-after-suspension)
    `(block ,retry-after-suspension
       (tagbody ,retry
          (return-from ,retry-after-suspension
            (%unwind-on-suspension #'(lambda () ,@body)
                                   #'(lambda () (go ,retry))))))))

(defmacro on-suspension (when-suspended-form &body body)
  "Executes `when-suspended-form' whenever a suspension event occurs
   while `form' is being executed.

   Non local exits out of `when-suspended-form' are prohibited."
  `(call-on-suspension #'(lambda () (assert-no-nlx ,when-suspended-form))
                       #'(lambda () ,@body)))



