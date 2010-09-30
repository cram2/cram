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


(in-package :cpm)

(defmethod initialize-instance :after ((pm process-module) &key
                                       (name (error "Process modules need a name."))
                                       (input (make-fluent
                                               :name (intern
                                                      (concatenate 'string
                                                                   (symbol-name name)
                                                                   "-INPUT"))))
                                       (feedback (make-fluent
                                                  :name (intern
                                                         (concatenate 'string
                                                                      (symbol-name name)
                                                                      "-FEEDBACK"))))
                                       (result (make-fluent
                                                :name (intern
                                                       (concatenate 'string
                                                                    (symbol-name name)
                                                                    "-RESULT"))))
                                       (cancel (make-fluent
                                                :name (intern
                                                       (concatenate 'string
                                                                    (symbol-name name)
                                                                    "-CANCEL"))))
                                       (status (make-fluent
                                                :name (intern
                                                       (concatenate 'string
                                                                    (symbol-name name)
                                                                    "-STATUS"))
                                                :value :offline))
                                       (caller (make-fluent
                                                :name (intern
                                                       (concatenate 'string
                                                                    (symbol-name name)
                                                                    "-CALLER")))))
  (setf (slot-value pm 'name) name
        (slot-value pm 'input) input
        (slot-value pm 'feedback) feedback
        (slot-value pm 'result) result
        (slot-value pm 'status) status
        (slot-value pm 'cancel) cancel
        (slot-value pm 'caller) caller))

(defmethod pm-run :around ((pm process-module))
  (assert (eq (value (slot-value pm 'status)) :offline) ()
          "Process module `~a' already running." pm)
  (setf (value (slot-value pm 'input)) nil
        (value (slot-value pm 'status)) :waiting
        (value (slot-value pm 'cancel)) nil)
  (with-slots (input cancel status result) pm
    (unwind-protect
         (block pm-body
           (whenever ((not (eq input nil)))
             (when (value input)
               (restart-case
                   (handler-bind ((error #'invoke-debugger))
                     (setf (value status) :running)
                     (setf (value cancel) nil)
                     (unwind-protect
                          (pursue
                            (wait-for cancel)
                            (handler-case
                                (handler-bind ((error (lambda (e)
                                                        (setf (value result) e)
                                                        (setf (value status) :failed))))
                                  (let ((result-val nil))
                                    (unwind-protect
                                         (when (value input)
                                           (setf result-val (call-next-method)))
                                      (setf (value input) nil)
                                      (when result-val
                                        (setf (value result) result-val)))))
                              (plan-error (e)
                                (declare (ignore e))
                                nil)))
                       (unless (eq (value status) :failed)
                         (setf (value status) :waiting))))
                 (terminate-pm ()
                   :report "Terminate process module"
                   (return-from pm-body))
                 (continue-pm ()
                   :report "Ignore error and restart process module main loop.")))))
      (setf (value status) :offline))))

(defmethod pm-run ((pm symbol))
  (pm-run (cdr (assoc pm *process-modules*))))

(defmethod pm-execute ((pm process-module) input &key
                       (async nil) (priority 0) (wait-for-free t)
                       (task *current-task*))
  ;; Note: priorities are unused currently.
  (with-slots (status result caller) pm
    (when (eq (value status) :offline)
      (warn "Process module ~a not running. Waiting for it to come up." pm)
      (wait-for (not (eq status :waiting))))
    (when (eq (value status) :running)
      (unless  wait-for-free
        (fail "Process module already processing an input."))
      (wait-for (not (eq status :running))))
    (setf (value caller) task)
    (setf (slot-value pm 'priority) priority)
    ;; Set the status to running here. Otherwise we might get a race
    ;; condition becaus status is not set to running yet and the next
    ;; wait-for returns immediately.
    (setf (value (slot-value pm 'input)) input)
    (wait-for (eq status :running))
    (unless async
      (unwind-protect
           (progn
             (wait-for (not (eq status :running)))
             (when (eq (value status) :failed)
               (error (value result))))
        (pm-cancel pm))
      (value result))))

(defmethod pm-cancel ((pm process-module))
  (setf (value (slot-value pm 'cancel)) t)
  ;; (wait-for (not (eq (slot-value pm 'status) :running)))
  )

(defmethod pm-cancel ((pm symbol))
  (pm-cancel (cdr (assoc pm *process-modules*))))

(defmethod pm-status ((pm process-module))
  (slot-value pm 'status))

(defmethod pm-status ((pm symbol))
  (pm-status (cdr (assoc pm *process-modules*))))
