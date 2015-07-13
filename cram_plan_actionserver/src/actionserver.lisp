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

(in-package :cram-actionserver)

(actionlib:def-exec-callback cram-actionserver-execute (plan parameters)
  (handler-case
      (let ((plan-symbol (cdr (assoc plan *plans* :test #'string-equal))))
        (unless plan-symbol
          (actionlib:abort-current result "Plan not found."))
        (let ((worker-thread nil)
              (result nil))
          (unwind-protect
               (progn
                 (setf worker-thread
                       (sb-thread:make-thread
                        (lambda ()
                          (handler-case
                              (setf result
                                    (apply (symbol-function plan-symbol)
                                           (let ((*read-eval* nil)
                                                 (*package* (symbol-package plan-symbol)))
                                             (map 'list #'read-from-string parameters))))
                            (condition (e)
                              (setf result e))))))
                 (loop-at-most-every 0.1
                   (when (actionlib:cancel-request-received)
                     (actionlib:abort-current))
                   (unless (sb-thread:thread-alive-p worker-thread)
                     (typecase result
                       (condition
                        (actionlib:abort-current :result (format nil "~a" result)))
                       (t (actionlib:succeed-current :result (format nil "~a" result)))))))
            (when (and worker-thread (sb-thread:thread-alive-p worker-thread))
              (let* ((task-tree (get-top-level-task-tree plan-symbol))
                     (code (task-tree-node-effective-code (cdr (car (task-tree-node-children task-tree))))))
                (assert code () "Task tree node doesn't contain a CODE. This is a bug.")
                (assert (code-task code) () "Task tree code doesn't contain a task. This is a bug.")
                (evaporate (code-task code)))))))
    (error (e)
      (actionlib:abort-current :result (format nil "~a" e)))))

(def-service-callback (plan-list cram_plan_actionserver-srv:planlist) ()
  (flet ((plan-description (plan-sym)
           (let ((doc-string (documentation (symbol-function plan-sym) 'function))
                 (lambda-list (get plan-sym 'cpl-impl::plan-lambda-list)))
             (format nil "~a~%Lambda list: ~a~%"
                     (or doc-string "No documentation available")
                     (or lambda-list "()")))))
    (make-response :plans (map 'vector #'car *plans*)
                   :descriptions (map 'vector (alexandria:compose #'plan-description #'cdr)
                                      *plans*))))

(defun maybe-setup-tracing ()
  (let ((tracing-enabled (get-param "~record_execution_trace" nil))
        (tracing-dir (get-param "~execution_trace_dir" nil)))
    (when tracing-enabled
      (if (not tracing-dir)
          (ros-fatal (actonserver cram)
                     "Recording of execution traces enabled but no directory specified. Please set the parameter execution_trace_dir.")
          (cet:setup-auto-tracing (make-pathname :directory tracing-dir) :ensure-directory t)))))

(defun server ()
  "Runs the actionserver and blocks."
  (unwind-protect
       (progn
         (setf cpl-impl:*break-on-plan-failures* nil)
         (setf cpl-impl:*debug-on-lisp-errors* nil)
         (roslisp-utilities:startup-ros :name "cram_actionserver" :anonymous nil)
         (register-service-fn "~list_plans" #'plan-list 'cram_plan_actionserver-srv:planlist)
         (maybe-setup-tracing)
         (actionlib:start-action-server
          "~execute_plan" "cram_plan_actionserver/ExecutePlanAction"
          #'cram-actionserver-execute))
    (roslisp-utilities:shutdown-ros)))
