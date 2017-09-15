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

(in-package :cram-process-modules)

(defclass asynchronous-process-module (abstract-process-module)
  ((input-queue :reader input-queue :initform nil
                :documentation "Designators that are waiting to be
                processed, i.e. for which ON-INPUT hasn't been called
                yet.")
   (processed-designators :reader processed-designators :initform nil
                          :documentation "Designators that haven't
                          been finished yet.")
   (failures :reader failures :initform nil
             :documentation "Sequence of designators and errors.")
   (queue-lock :reader queue-lock :initform (sb-thread:make-mutex))
   (input-lock :reader input-lock :initform (sb-thread:make-mutex))
   (notification-fluent :reader notification-fluent)))

(defmacro def-asynchronous-process-module (name direct-slots &rest options)
  "Defines a new asynchronous process module. The macro wraps around
  DEFCLASS and should be used to create new asynchronous process
  modules. `name' is the name of the process module to be defined. The
  `direct-slots' and `options' parameters are directly passed to
  DEFCLASS."
  `(progn
     (defclass ,name (asynchronous-process-module) ,direct-slots ,@options)
     (register-process-module ',name)))

(defmethod initialize-instance :after ((process-module asynchronous-process-module) &key)
  (with-slots (name notification-fluent) process-module
    (setf notification-fluent
          (cpl-impl:make-fluent
           :name (intern
                  (concatenate 'string (symbol-name name) "-NOTIFICATION"))))))

;;; Methods that need to be implemented by the user.
(defgeneric on-input (process-module input-designator)
  (:documentation "Method that is executed whenever new input is
  received. ON-INPUT invocations are always sequential, i.e. they
  never happen before the previous ON-INPUT returned."))

(defgeneric on-cancel (process-module input-designator)
  (:documentation "Is called when cancellation is requested. The
  default implementation does nothing.")
  (:method ((process-module asynchronous-process-module) input-designator)
    nil))

(defgeneric on-run (process-module)
  (:documentation "Method that is executed when the process module is
  started. When it blocks, this method will be unwound when the
  process module is shut down. It can be used to implement the process
  module's worker thread. Note that in rare cases, it can happen that
  ON-INPUT is called before ON-RUN when input is received right after
  the process module has been started.")
  (:method ((process-module asynchronous-process-module))
    nil))

(defgeneric synchronization-fluent (process-module designator)
  (:documentation "Returns a fluent indicating if PM-EXECUTE needs to
  block or can immediately return when called with `designator'. In
  other words, PM-EXECUTE always first calls WAIT-FOR on the result of
  this method.

  The idea is that, for instance, navigation needs to block until
  manipulation is finished, manipulation needs to block until
  navigation is finished but manipulation doesn't need to block if
  only one arm is needed and one is still free."))

;;; API Methods that can be used by the user.
(defgeneric finish-process-module (process-module &key designator)
  (:documentation "Signals that the process module finished
  processing. `designator' can be specified optionally and indicates
  that only this designator has finished.")
  (:method ((process-module symbol) &key designator)
    (finish-process-module
     (get-running-process-module process-module) :designator designator)))

(defgeneric fail-process-module (process-module error &key designator)
  (:documentation "Similar to FINISH but causes all monitor methods
  monitor the execution of `designator' to rethrow `error'. If
  `designator' is unspecified, fails for all designators that are
  currently processed.")
  (:method ((process-module symbol) error &key designator)
    (fail-process-module
     (get-running-process-module process-module)
     error :designator designator)))

(defmethod pm-run ((process-module asynchronous-process-module) &optional name)
  (assert (eq (value (slot-value process-module 'status)) :offline) ()
          "Process module `~a' already running." process-module)
  (with-slots (input-queue processed-designators notification-fluent
               input status cancel queue-lock)
      process-module
    (let ((name (or name (class-name (class-of process-module)))))
      (with-process-module-registered-running (name process-module)
        (setf (value input) nil
              (value status) :waiting
              (value cancel) nil)
        (unwind-protect
             (par
               (on-run process-module)
               (whenever (input)
                 (setf (value status) :running)
                 (setf (value cancel) nil)
                 (let ((input-value (value input)))
                   (sb-thread:with-mutex (queue-lock)
                     (if input-queue
                         (setf (cdr (last input-queue)) (list input-value))
                         (setf input-queue (list input-value))))
                   (pulse notification-fluent)
                   (setf (value input) nil)))
               (whenever ((pulsed notification-fluent
                                  :handle-missed-pulses :always))
                 (let ((input-value
                         (sb-thread:with-mutex (queue-lock)
                           (when input-queue
                             (push (car input-queue) processed-designators)
                             (setf input-queue (cdr input-queue))
                             (car processed-designators)))))
                   (when input-value
                     (on-input process-module input-value)
                     (pulse notification-fluent)
                     (on-process-module-started process-module input-value))))
               ;; TODO(moesenle): add handling of cancellation on a
               ;; per-designator basis.
               (whenever (cancel)
                 (sb-thread:with-mutex (queue-lock)
                   (dolist (designator processed-designators)
                     (on-cancel process-module designator)))))
          (setf (value status) :offline))))))

(defmethod pm-execute ((process-module asynchronous-process-module) input-designator
                       &key (task cpl-impl:*current-path*))
  (with-slots (input status caller input-lock) process-module
    (assert (not (eq (value status) :offline)) () "Process module ~a not running."
            process-module)
    (sb-thread:with-mutex (input-lock)
      ;; If the process module is receiving input already, wait.
      (wait-for (cpl:not input))
      (wait-for (synchronization-fluent process-module input-designator))
      (setf (value caller) task)
      (setf (value input) input-designator))))

(defmethod finish-process-module ((process-module asynchronous-process-module)
                                  &key designator)
  (with-slots (status processed-designators queue-lock notification-fluent) process-module
    (assert (eq (value status) :running) ()
            "Cannot finish an already stopped process module.")
    (sb-thread:with-mutex (queue-lock)
      (cond (designator
             (assert (find designator processed-designators) ()
                     "Cannot finish a designator that is not being processed: ~a"
                     designator)
             (setf processed-designators (remove designator processed-designators)))
            (t (setf processed-designators nil)
               (setf (value status) :waiting)))
      (pulse notification-fluent)))
  (on-process-module-finished process-module designator nil))

(defmethod fail-process-module ((process-module asynchronous-process-module) error
                                &key designator)
  (with-slots (processed-designators failures queue-lock notification-fluent)
      process-module
    (sb-thread:with-mutex (queue-lock)
      (cond (designator
             (assert (find designator processed-designators) ()
                     "Cannot fail for designator that is not processed: ~a"
                     designator)
             (setf processed-designators (remove designator processed-designators))
             (push (cons error designator) failures))
            (t (dolist (designator processed-designators)
                 (push (cons error designator) failures))
               (setf processed-designators nil)))
      (pulse notification-fluent)))
  (on-process-module-failed process-module designator error))

(defmethod monitor-process-module ((process-module asynchronous-process-module)
                                   &key designators)
  (with-slots (notification-fluent queue-lock failures input
               input-queue processed-designators)
      process-module
    (flet ((check-finished-with-designators (designators)
             (let ((error-info (find-if (lambda (error-info)
                                          (member (cdr error-info) designators))
                                        failures)))
               (when error-info
                 (setf failures (remove (car error-info) failures
                                        :key #'car))
                 (error (car error-info))))
             (some (lambda (designator)
                     (not (or (eq designator (value input))
                              (member designator input-queue)
                              (member designator processed-designators))))
                   designators))
           (check-all-finished ()
             (when failures
               (let ((error-info (car failures)))
                 (setf failures (cdr failures))
                 (error (car error-info))))
             (not (or (value input) processed-designators input-queue))))
      (let ((notification (pulsed notification-fluent :handle-missed-pulses :once))
            (designators (typecase designators
                           (list designators)
                           (t (list designators)))))
        ;; TODO(moesenle): add handling of suspension/non-local exits
        ;; to cancel and maybe restart the action.
        (loop do
          (sb-thread:with-mutex (queue-lock)
            (when (if designators
                      (check-finished-with-designators designators)
                      (check-all-finished))
              (return nil)))
          (wait-for notification))))))
