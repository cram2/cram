;;;
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
;;;

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

(define-condition unknown-process-module (simple-error) ())

(defclass process-module ()
  ((name :reader name :documentation "The name of the process-module")
   (input :reader input :documentation "Input fluent.")
   (feedback :reader feedback :documentation "Feedback fluent.")
   (result :reader result :documentation "Result fluent")
   (status :reader status
           :documentation "Status fluent. Possible status: 
                           :waiting :running :failed")
   (cancel :reader cancel :documentation "Cancel request fluent")
   (priority :reader priority :initform nil
             :documentation "Priority of the currently running
                             process. Only higher priorized processes
                             can send new inputs while the process
                             module is running.")
   (caller :reader caller
           :documentation "Fluent containing the task that sent the
                           current input.")))

(defgeneric pm-run (process-module)
  (:documentation "Represents the main event loop of the process
                   Module. Note: pm-run will never return (due to an
                   around method). Otherwise, the process module would
                   be dead. When a cancel is triggered, the pm-run method
                   is evaporated and restarted. status and result are
                   set implicitly. pm-run must not set these
                   values. Feedback can be used to provide feedback
                   information."))

(defgeneric pm-execute (process-module input &key async priority wait-for-free task)
  (:documentation "Executes a process module.  

                   async: return immediately after sending the input
                          and triggering execution.
                   priority: priority of the task triggering the pm.
                   wait-for-free: if already running wait for exit and send input.
                   task: the task triggering the pm."))

(defgeneric pm-cancel (process-module))

(defgeneric pm-status (process-module))

(cpl-impl:define-task-variable *process-modules* nil)

(defmacro def-process-module (name (desig-var) &body body)
  (with-gensyms (pm-var-name)
    `(progn
       (defclass ,name (process-module) ())
       (defmethod pm-run ((,pm-var-name ,name))
         (block nil
           (let ((,desig-var (current-desig (value (slot-value ,pm-var-name 'input)))))
             ,@body)))
       (eval-when (:load-toplevel)
         (cond ((assoc ',name *process-modules*)
                #+sbcl (sb-int:style-warn "Redifining process module `~a'" ',name)
                (setf (cdr (assoc ',name *process-modules*))
                      (make-instance ',name :name ',name)))
               (t
                (push (cons ',name (make-instance ',name :name ',name))
                      *process-modules*)))))))

(defmethod pm-execute ((pm symbol) input &key
                       (async nil) (priority 0) (wait-for-free t)
                       (task *current-task*))
  (let ((pm-known (cdr (assoc pm *process-modules*)) ))
    (unless pm-known
            (error 'unknown-process-module :format-control "Unknown process module: ~a "  :format-arguments (list pm)))
    (pm-execute pm-known input
     :async async :priority priority
     :wait-for-free wait-for-free :task task)))

(defun process-module-alias (alias name)
  "Allows for the definition of process module aliases. An alias is
just a different name for an existing process module."
  (push (cons alias (cdr (assoc name *process-modules*)))
        *process-modules*))

(defmacro with-process-module-aliases (alias-definitions &body body)
  "Executes body with process module aliases bound in the current
  task-dynamic environment. `alias-definitions' has the form

  (WITH-PROCESS-MODULE-ALIASES ((<alias> <name>)*) <body-form>*)"

  `(let ((*process-modules*
          (list* ,@(mapcar (lambda (def)
                             `(cons ',(car def) (cdr (assoc ',(cadr def) *process-modules*))))
                           alias-definitions)
                *process-modules*)))
     ,@body))
