;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :cram-projection)

(defvar *projection-environments* nil
  "List of projection environments.")

(defvar *special-projection-variables* nil
  "List of special variables that need to be rebound for projection.")

(cpl-impl:define-task-variable *projection-environment* nil
  "When projecting, this variable contains the name of the current
  projection environment.")

(defstruct projection-environment
  name
  function)

(defstruct projection-environment-result
  "Data structure returned by WITH-PROJECTION-ENVIRONMENT. It contains
  the name of the projection environment, the list of result values
  and the bindings of all global and local special projection
  variables."
  name
  result
  environment)

(defun execute-in-projection-environment (projection-environment function)
  "Executes `function' in the environment defined by
  `projection-environment'."
  (funcall (projection-environment-function projection-environment)
           function))

(defun make-projection-environment-function
    (name special-variable-initializers process-module-definitions startup shutdown)
  `(lambda (function)
     (let ((*projection-environment* ',name)
           ,@special-variable-initializers)
       (cram-process-modules:with-process-modules-running ,process-module-definitions
         (unwind-protect
              (progn
                ,startup
                (make-projection-environment-result
                 :name *projection-environment*
                 :result (multiple-value-list (funcall function))
                 :environment (symbol-values-alist
                               (append ',(mapcar #'car special-variable-initializers)
                                       (mapcar #'car *special-projection-variables*)))))
           ,shutdown)))))

(defun symbol-values-alist (variable-names)
  "Returns an alist for all symbol values in `variable-names' of the
  form (variable-name . (symbol-value variable-name))."
  (mapcar (lambda (name)
            (cons name (symbol-value name)))
          variable-names))

(defmacro define-projection-environment
    (name &key special-variable-initializers process-module-definitions
            startup shutdown)
  "For projecting a plan, some special variables might need to be
  rebound. E.g. TF, things like data caches for the environment
  etc. need at least to be copied to not infer with current plan
  execution. In addition, different process modules need to be used
  instead of the actual process modules that are sending commands to
  the robot.

  The macro `def-projection-environment' creates a new projection
  environment. This environment contains enough information to set up
  dynamic bindings, process modules etc. to execute a plan in
  projection mode.

  Parameters:

  `name' The name of the environment. The name is used to re-define or
  reference the projection environment in
  WITH-PROJECTION-ENVIRONMENT.

  `special-variable-initializers' A list of initializers for special
  variables of the form \(variable-name initialization-form\)*. All
  variables are re-defined to be task-variables according to the
  definition of task variables created by DEFINE-TASK-VARIABLE. Before
  projection is started, the task variables are initialized by
  executing `initialization-form'. This happens before executing
  `startup'. When projection is started,
  `special-variable-initializers' and all special projection variables
  defined by DEFINE-SPECIAL-PROJECTION-VARIABLE are appended.

  `process-module-definitions'  A list of process module definitions
  similar to WITH-PROCESS-MODULES-RUNNING. It defines the process
  modules that should be used for projecting a plan. They are started
  before `startup' is executed.

  `startup'  A form that is executed just before projection is started,
  but after all initializations finished, i.e. after all special
  variables have been re-bound and after the process modules are
  started.

  `shutdown' A form that is executed right after projection has
  finished but before the process modules are stopped."
  `(eval-when (:compile-toplevel :load-toplevel :execute)
     ,@(mapcar (lambda (variable-definition)
                 `(cpl:define-task-variable ,(car variable-definition)))
               special-variable-initializers)
     (setf *projection-environments*
           (cons (make-projection-environment
                  :name ',name
                  :function ,(make-projection-environment-function
                              name
                              special-variable-initializers
                              process-module-definitions
                              startup shutdown))
                 (remove ',name *projection-environments*
                         :key #'projection-environment-name)))))

(defmacro define-special-projection-variable (name initializer)
  "Defines a special variable that needs to be re-initialized for
projection mode. Note: DEFINE-SPECIAL-PROJECTION-VARIABLE defines
variables for all environments. `name' is the name of the special
variable and `initializer' is a form that is evaluated at
initialization time of projection. `name' is defined as a task
variable according to CPL:DEFINE-TASK-VARIABLE."
  `(eval-when (:compile-toplevel :load-toplevel :execute)
     (cpl:define-task-variable ,name)
     (setf *special-projection-variables*
           (cons (cons ',name (lambda () ,initializer))
                 (remove ',name *special-projection-variables*
                         :key #'car)))))

(defmacro with-projection-environment (name &body body)
  `(flet ((body-function ()
            (progv (mapcar #'car *special-projection-variables*)
                (mapcar (alexandria:compose #'funcall #'cdr)
                        *special-projection-variables*)
              ,@body)))
     (let ((environment (find ',name *projection-environments*
                              :key #'projection-environment-name)))
       (unless environment
         (error 'simple-error
                :format-control "Cannot find projection environment `~a'."
                :format-arguments (list ',name)))
       (execute-in-projection-environment environment #'body-function))))
