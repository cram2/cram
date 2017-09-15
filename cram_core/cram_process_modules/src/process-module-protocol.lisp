;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :cpm)

;;; Process modules
;;;
;;; A process module encapsulates interaction with the outer
;;; world. Implementing a new process module is rather easy:
;;; 
;;; (def-process-module test-pm (pm)
;;;   (format t "received input ~a~%" (input pm))
;;;
;;; Use pm-execute to send input to a running pm and use pm-cancel to
;;; cancel execution.
;;;
;;; Handling of priorities is not implemented yet.

(define-task-variable *process-module-debugger-hook* nil
  "Hook that is called when an error is thrown in the process
  module. If NIL, *DEBUGGER-HOOK* is used.")

(define-condition unknown-process-module (simple-error) ())
(define-condition process-module-running (simple-error) ())
(define-condition process-module-not-running (simple-error) ())

(defclass abstract-process-module ()
  ((name :reader name :documentation "The name of the process-module"
         :initform (error "Process modules need a name.")
         :initarg :name)
   (input :reader input :documentation "Input fluent.")
   (status :reader status
           :documentation "Status fluent. Possible status: 
                           :waiting :running :failed")
   (cancel :reader cancel :documentation "Cancel request fluent")
   (caller :reader caller
           :documentation "Fluent containing the task that sent the
                           current input.")))

(defmethod initialize-instance :after ((pm abstract-process-module) &key)
  ;; We cannot initialize the slots inside the class because we need
  ;; the name of the module to construct the fluent names.
  (flet ((make-fluent-name (process-module-name type)
           (declare (type symbol process-module-name type))
           (intern (concatenate
                    'string (symbol-name process-module-name)
                    "-" (symbol-name type)))))
    (with-slots (name input status cancel caller)
        pm
      (setf input (make-fluent :name (make-fluent-name name :input)))
      (setf status (make-fluent :name (make-fluent-name name :status)
                                :value :offline))
      (setf cancel (make-fluent :name (make-fluent-name name :cancel)))
      (setf caller (make-fluent :name (make-fluent-name name :caller))))))

(defgeneric pm-run (process-module &optional name)
  (:documentation "Represents the main event loop of the process
  Module. Note: pm-run will never return (due to an around
  method). Otherwise, the process module would be dead. When a cancel
  is triggered, the pm-run method is evaporated and restarted. status
  and result are set implicitly. pm-run must not set these
  values. Parameters:

  `process-module' The process module to run, either an CLOS instance
  or a symbol naming the CLOS class. If an instance is passed and the
  `name' parameter is unbound, the class name is used for naming the
  process module.

  `name' is an optional name of the process-module if the name should
  not be the name of the CLOS class."))

(defgeneric pm-execute (process-module input &key task)
  (:documentation "Executes a process module. `task' should should be
  set to the task that is calling PM-EXECUTE."))

(defgeneric pm-cancel (process-module))

(defgeneric pm-status (process-module))

(defgeneric running-fluent (process-module)
  (:documentation "Convenience method that returns a fluent that is T
  when the process module is running and NIL otherwise."))

(defgeneric finished-fluent (process-module)
  (:documentation "Convenience method that returns a fluent that is T
  when the process module is finished, NIL otherwise."))

(defgeneric monitor-process-module (process-module &key designators)
  (:documentation "Monitors the execution of a process module. The
  method blocks until the process module has finished processing or
  throws an error. In that case rethrows the error. When `designators'
  is set, only monitors the execution of `designators'. Otherwise,
  monitors all failures. `designators' can be a single designators or
  a list of designators.")
  (:method ((process-module symbol) &key designators)
    (monitor-process-module
     (get-running-process-module process-module) :designators designators)))

(defclass process-module-collection ()
  ((process-modules :initform nil :accessor process-modules)
   (lock :initform (sb-thread:make-mutex))
   (condition :initform (sb-thread:make-waitqueue))))

(defvar *process-modules* nil
  "The sequence of all registered process modules.")

(cpl-impl:define-task-variable *running-process-modules*
    (make-instance 'process-module-collection)
    "The list of currently running process modules.")

(defmethod pm-cancel ((pm symbol))
  (pm-cancel (get-running-process-module pm)))

(defmethod pm-status ((pm symbol))
  (pm-status (get-running-process-module pm)))

(defmethod pm-run ((process-module symbol) &optional name)
  (unless (find process-module *process-modules*)
    (error 'unknown-process-module
           :format-control "Unknown process module: ~a "
           :format-arguments (list process-module)))
  (when (check-process-module-running process-module :throw-error nil)
    (error 'process-module-running
           :format-control "Process module `~a' already running. foo foo"
           :format-arguments (list process-module)))
  (pm-run (make-instance process-module :name name) name))

(defmethod pm-execute ((pm symbol) input &key (task *current-task*))
  (let ((pm-known (get-running-process-module pm)))
    (unless pm-known
      (error 'unknown-process-module
             :format-control "Unknown process module: ~a "
             :format-arguments (list pm)))
    (pm-execute pm-known input :task task)))

(defmethod pm-execute :before ((process-module abstract-process-module) input &key task)
  (declare (ignore task))
  (with-slots (status) process-module
    (when (eq (value status) :offline)
      (warn "Process module ~a not running. Status is ~a. Waiting for it to come up."
            process-module (value status))
      (wait-for (not (eq status :offline))))))

(cut:define-hook on-process-module-started (module input)
  (:documentation "Hook that is called whenever the process module
  `module' receives `input' and starts executing."))

(cut:define-hook on-process-module-finished (module input result)
  (:documentation "Hook that is called whenever the process module
  `module' finishes."))

(cut:define-hook on-process-module-failed (module input failure)
  (:documentation "Hook that is called whenever the process module
  `module' throws an error."))

(defun register-process-module (name)
  (pushnew name *process-modules*))

(defun get-process-module-names ()
  *process-modules*)

(defun get-running-process-module (name)
  "Returns the process module that corresponds is named `name'. `name'
  can also be an alias. Returns NIL if the module could not be found"
  (cdr (assoc name (process-modules *running-process-modules*))))

(defun get-running-process-module-name (module)
  "Returns the name of the process module indicated by `module' or NIL
  if the module is not registered"
  (declare (type abstract-process-module module))
  (car (rassoc module (reverse (process-modules *running-process-modules*)))))

(defun get-running-process-module-names (module)
  "Returns the list of all aliases for `module'"
  (declare (type abstract-process-module module))
  (mapcar #'car (remove-if-not
                 (lambda (i) (eql i module))
                 (process-modules *running-process-modules*) :key #'cdr)))

(defun process-module-alias (alias name)
  "Allows for the definition of process module aliases. An alias is
just a different name for an existing process module."
  (let ((module (get-running-process-module name)))
    (unless module
      (error 'unknown-process-module
             :format-control "Could not find process module with name `~s'"
             :format-arguments (list name)))
    (push (cons alias module) (process-modules *running-process-modules*))))

(defun check-process-module-running (process-module &key (throw-error t))
  (declare (type (or symbol abstract-process-module) process-module))
  (or
   (find process-module (process-modules *running-process-modules*)
         :key (etypecase process-module
                (symbol #'car)
                (abstract-process-module #'cdr)))
   (when throw-error
     (error 'process-module-not-running
            :format-control "Process module `~a' not running."
            :format-arguments (list process-module)))))

(defun set-process-module-running (name instance)
  (declare (type symbol name)
           (type abstract-process-module instance))
  (when (or (not (eql (value (status instance)) :offline))
            (check-process-module-running name :throw-error nil))
    (error 'process-module-running
           :format-control "Process module `~a' already running"
           :format-arguments (list name)))
  (with-slots (lock condition process-modules) *running-process-modules*
    (sb-thread:with-mutex (lock)
      (push (cons name instance) process-modules)
      (sb-thread:condition-broadcast condition))))

(defun remove-process-module-running (process-module)
  (declare (type (or symbol abstract-process-module) process-module))
  (with-slots (lock condition process-modules) *running-process-modules*
    (sb-thread:with-mutex (lock)
      (setf process-modules
            (remove process-module process-modules
                    :key (etypecase process-module
                           (symbol #'car)
                           (abstract-process-module #'cdr))))
      (sb-thread:condition-broadcast condition))))

(defun wait-for-process-module-running (process-module &key timeout)
  (flet ((timer-function ()
           (return-from wait-for-process-module-running nil))
         (wait-for-running ()
           (with-slots (lock condition process-modules)
               *running-process-modules*
             (loop until (check-process-module-running
                          process-module :throw-error nil)
                   do (sb-thread:with-mutex (lock)
                        (sb-thread:condition-wait condition lock))
                   finally (return-from wait-for-process-module-running t)))))
    (let ((timer (sb-ext:make-timer #'timer-function)))
      (cond (timeout
             (sb-ext:schedule-timer timer timeout)
             (unwind-protect
                  (wait-for-running)
               (sb-ext:unschedule-timer timer)))
            (t (wait-for-running))))))

(defmacro with-process-module-aliases (alias-definitions &body body)
  "Executes body with process module aliases bound in the current
  task-dynamic environment. `alias-definitions' has the form

  (WITH-PROCESS-MODULE-ALIASES ((<alias> <name>)*) <body-form>*)"

  `(let ((*process-modules*
           (list* ,@(mapcar
                     (lambda (def)
                       `(cons ',(car def)
                              (cdr
                               (assoc
                                ',(cadr def)
                                (process-modules *running-process-modules*)))))
                     alias-definitions)
                  (process-modules *running-process-modules*))))
     ,@body))

(defmacro with-process-module-registered-running ((name instance) &body body)
  "Makes sure that the process module is running. If not, registers
  the process module under `name' as running."
  `(unwind-protect
        (progn
          (unless (check-process-module-running ,name :throw-error nil)
            (set-process-module-running ,name ,instance))
          ,@body)
     (remove-process-module-running ,name)))

(defmacro with-process-modules-running (process-modules &body body)
  "Runs all process modules specified in `process-modules' in a
  separate thread and executes `body'. Terminates all process modules
  after `body' finished. 

  `process-modules' is a list with elements with the following format:

  PROCESS-MODULE | (NAME PROCESS-MODULE)

  Example:
  (with-process-modules-running 
      (foo
       (:bar baz))
    (code))
  
  In the example, the two process modules foo and baz are started
  up. baz is run with name :bar.

  Note: This macro uses PURSUE defined in cram_language and therefore
  needs to be run in a TOP-LEVEL form."
  (let ((process-module-definitions (mapcar
                                     (lambda (definition)
                                       (etypecase definition
                                         (symbol (list definition definition))
                                         (list definition)))
                                     process-modules)))
    `(flet ((body-function () ,@body))
       ,(if process-modules
            `(flet ((process-modules-thread (running-fluent)
                      (top-level
                        (pursue
                          (par ,@(loop for (alias name) in process-module-definitions
                                       collecting `(pm-run ',name ',alias)))
                          (seq
                            (par ,@(loop for (alias name) in process-module-definitions
                                         collecting `(wait-for-process-module-running ',alias)))
                            (setf (value running-fluent) t)
                            (wait-for (not running-fluent)))))))
               (let ((process-modules-running (make-fluent :value nil)))
                 (sb-thread:make-thread
                  (tv-closure nil nil
                              (lambda ()
                                (process-modules-thread process-modules-running))))
                 (wait-for process-modules-running)
                 (unwind-protect (body-function)
                   (setf (value process-modules-running) nil))))
            `(body-function)))))
