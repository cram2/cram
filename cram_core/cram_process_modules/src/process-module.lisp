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

(in-package :cpm)

(defclass process-module (abstract-process-module)
  ((execute-lock :initform (sb-thread:make-mutex))
   (result :reader result :documentation "Result fluent")
   (unprocessed-results
    :reader unprocessed-results
    :documentation "Weak hash table from input designators to
    results. Only the input designator is weak."
    :initform (make-hash-table :weakness :key :synchronized t))))

(defmethod initialize-instance :after ((process-module process-module) &key)
  (with-slots (name result) process-module
    (setf result (make-fluent
                  :name (intern
                         (concatenate
                          'string (symbol-name name) "-RESULT"))))))

(defmacro def-process-module (name (desig-var) &body body)
  (with-gensyms (pm-var-name name-variable)
    `(progn
       (defclass ,name (process-module) ())
       (defmethod pm-run ((,pm-var-name ,name) &optional ,name-variable)
         (declare (ignore ,name-variable))
         (block nil
           (let ((,desig-var (current-desig (value (slot-value ,pm-var-name 'input)))))
             ,@body)))
       (register-process-module ',name))))

(defmethod pm-run :around ((pm process-module) &optional name)
  (assert (eq (value (slot-value pm 'status)) :offline) ()
          "Process module `~a' already running." pm)
  (let ((name (or name (class-name (class-of pm)))))
    (with-process-module-registered-running (name pm)
      (setf (value (slot-value pm 'input)) nil
            (value (slot-value pm 'status)) :waiting
            (value (slot-value pm 'cancel)) nil)
      (let ((teardown nil))
        (with-slots (input cancel status result unprocessed-results) pm
          (unwind-protect
               (block pm-body
                 (loop do
                   (wait-for (not (eq input nil)))
                   (setf (value status) :running)
                   (setf (value cancel) nil)
                   (unwind-protect
                        (pursue
                          (wait-for cancel)
                          (restart-case
                              (handler-bind ((error
                                               (lambda (e)
                                                 (let ((*debugger-hook*
                                                         (or *process-module-debugger-hook*
                                                             *debugger-hook*)))
                                                   (funcall #'invoke-debugger e)))))
                                (handler-case
                                    (let ((input-value (value input))
                                          (result-value nil))
                                      (flet ((handle-failure (e)
                                               (setf (value result) e)
                                               (setf (value status) :failed)
                                               (on-process-module-failed pm input-value e)))
                                        (handler-bind ((plan-failure #'handle-failure)
                                                       (error #'handle-failure))
                                          (unwind-protect
                                               (progn
                                                 (assert
                                                  input-value ()
                                                  "Input value is NIL when calling process module. This should never happen.")
                                                 (on-process-module-started pm input-value)
                                                 (setf result-value (multiple-value-list (call-next-method))))
                                            (when result-value
                                              (setf (value result) result-value)
                                              (setf (gethash input-value unprocessed-results) result-value))
                                            (setf (value input) nil)
                                            (on-process-module-finished pm input-value (value result))))))
                                  (plan-failure (e)
                                    (declare (ignore e))
                                    nil)))
                            (terminate-pm ()
                              :report "Terminate process module"
                              ;; We need to use a flag here because we
                              ;; cannot perform a non-local exit to the
                              ;; block pm-body here. The reason is that we
                              ;; are inside a pursue, i.e. we are actually
                              ;; running in a different thread
                              (setf teardown t))
                            (continue-pm ()
                              :report "Ignore error and restart process module main loop.")))
                     (unless (eq (value status) :failed)
                       (setf (value status) :waiting)))
                   (when teardown
                     (return-from pm-body))))
            (setf (value status) :offline)))))))

(defmethod pm-execute ((pm process-module) input &key (task *current-task*))
  (with-slots ((input-fluent input)
               status result caller
               execute-lock) pm
    (retry-after-suspension
      (unwind-protect
           (progn
             (sb-thread:with-mutex (execute-lock)
               (when (eq (value status) :running)
                 (warn "Process module ~a already processing input. Waiting for it to become free."
                       pm)
                 (wait-for (not (eq status :running))))    
               (setf (value caller) task)
               (setf (value input-fluent) input)
               ;; Wait for the input fluent to be cleared again. That indicates
               ;; that the process module started processing it. Please note
               ;; that waiting for the status to become :running can lead to
               ;; race conditions if the process module immediately returns.
               (wait-for (not input-fluent)))
             (wait-for (not (eq status :running)))
             (when (eq (value status) :failed)
               (error (value result)))
             (apply #'values (value result)))
        (pm-cancel pm)))))

(defmethod pm-cancel ((pm process-module))
  (setf (value (slot-value pm 'cancel)) t)
  ;; (wait-for (not (eq (slot-value pm 'status) :running)))
  )

(defmethod pm-status ((pm process-module))
  (slot-value pm 'status))

(defmethod running-fluent ((process-module process-module))
  (eq (slot-value process-module 'status) :running))

(defmethod finished-fluent ((process-module process-module))
  (not (running-fluent process-module)))

(defmethod monitor-process-module ((process-module process-module) &key designators)
  ;; Since the actual pm-execute call already blocks and rethrows
  ;; errors, we only wait for the process module to finish in case it
  ;; is running.
  ;;
  ;; NOTE(moesenle): The implementation here is less than perfect, it
  ;; might suffer from different race conditions. For instance, if one
  ;; process calls PM-EXECUTE and right after calls
  ;; MONITOR-PROCESS-MODULE and another process calls PM-EXECUTE right
  ;; in between, the monitor function will block on the other process'
  ;; input.
  ;;
  ;; NOTE(moesenle): this is not really clean. I guess we need to
  ;; either check if a condition has been handled in pm-execute and
  ;; rethrow it in case it has not been handled or we need to rethrow
  ;; errors in any case. Not sure what's right here...
  (flet ((get-and-remove-first-matching-result (designators)
           "Iterates over `designators' until one is present in the
           process module's slot UNPROCESSED-RESULT, removes the value
           and returns it. If no designator could be found, returns
           NIL."
           (with-slots (unprocessed-results) process-module
             (dolist (designator designators)
               (let ((result (gethash designator unprocessed-results)))
                 (when result
                   (remhash designator unprocessed-results)
                   (return result))))))
         (get-and-remove-first-result ()
           "Returns the first result stored in the results hash table."
           (with-slots (unprocessed-results) process-module
             (loop for key being the hash-keys in unprocessed-results
                   using (hash-value value)
                   do (remhash key unprocessed-results)
                      (return value)))))
    (loop do
      (wait-for (finished-fluent process-module))
      (let ((result (if designators
                        (get-and-remove-first-matching-result designators)
                        (get-and-remove-first-result))))
        (when result
          (return (apply #'values result))))
      (wait-for (pulsed (finished-fluent process-module))))))
