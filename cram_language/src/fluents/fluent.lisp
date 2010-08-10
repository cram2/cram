;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>,
;;;                     Nikolaus Demmel <demmeln@cs.tum.edu>
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

(defclass fluent ()
  ((name :initarg :name :initform (error "No name specified.")
         :reader name :type symbol
         :documentation "The name of the fluent. Should be a globally unique symbol.")
   (on-update :initform (make-hash-table :test 'cl:eq) :type hash-table
              :documentation "Hash-table of update callbacks, being executed on every change of
                              the value. The update callback is a function with no parameters.")
   (pulse-count :initarg :pulse-count :initform 0
                :documentation "For internal use. Indicates a pulse.")
   (changed-condition :documentation "For internal use. Posix condition variable/waitqueue
                                      used for notification when value changed.")
   (value-lock :documentation "For internal use. Lock to be used with for access synchronization.")))

(defclass fl-cacheable-value-mixin () ()
  (:documentation "This mixin indicates that the value of this fluent
  can be cached, i.e. it doesn't change wihtout sending a pulse."))

(defclass fl-printable-mixin () ()
  (:documentation "This mixin indicates the value of the fluent can be
  printed by the pretty-printer. This is not the case for PULSE
  fluents, for instance."))

(defgeneric value (fluent)
  (:documentation "Reader method, returning the fluent's value"))

(defgeneric register-update-callback (fluent name update-fun)
  (:documentation "Method to register an update callback under the corresponding name.
                   When the name is already known, an error is signaled."))

(defgeneric remove-update-callback (fluent name)
  (:documentation "Method to remove the update callback with the given name."))

(defgeneric wait-for (fluent &key timeout wait-status)
  (:documentation
   "Method to block the current thread until the value of `fluent'
    becomes non-nil. If `timeout' is specified, waits for at least
    timeout and returns. The parameter `wait-status' indicates the
    status of the task when it is waiting."))

(defgeneric pulse (fluent)
  (:documentation "Method to trigger the fluent, i.e. notifying all waiting threads,
                   but without actually changing the fluent value."))

(defmacro with-fluent-locked (fluent &body body)
  `(with-unsuspendable-lock (slot-value ,fluent 'value-lock)
     ,@body))

(defvar *peek-value* nil "This is used to implement PEEK-VALUE.")

(defun peek-value (fluent)
  "Return the same value as VALUE, except that this never changes the state of
   the flunet. E.g. for PULSE-FLUENT the internal counter is not incremented."
  (let ((*peek-value* t))
    (value fluent)))

(defmethod value (var)
  "Default handler. It seems to be a quite good idea to allow the
  usage of value for all types of objects."
  var)

(defmethod wait-for (var &key &allow-other-keys)
  (or var
      (error 'simple-error :format-control "wait-for called with nil argument.")))

(defmethod initialize-instance :after ((fluent fluent) &key)
  (with-slots (name changed-condition value-lock) fluent
    (setf value-lock (make-recursive-lock :name (format nil "~a-lock" name)))
    (setf changed-condition (make-condition-variable :lock value-lock))))

(defmethod print-object ((fluent fl-printable-mixin) stream)
  (print-unreadable-object (fluent stream :type t :identity t)
    (format stream "[~s]" (value fluent))))

(defmethod register-update-callback ((fluent fluent) name update-fun)
  (with-fluent-locked fluent
    (if (gethash name (slot-value fluent 'on-update))
        (error "Callback with name ~a is already registered." name)
        (setf (gethash name (slot-value fluent 'on-update)) update-fun))))

(defmethod remove-update-callback ((fluent fluent) name)
  (with-fluent-locked fluent
    (remhash name (slot-value fluent 'on-update))))

(defmethod wait-for ((fluent fluent) &key
                     (timeout nil) (wait-status :waiting))
  (with-slots (changed-condition value-lock pulse-count) fluent
    (with-status (wait-status)
      (let ((start-time (get-internal-real-time))
            ;; Make sure that the value method is evaluated only once
            ;; per iteration. This is because pulsed-fluent keeps its
            ;; truth value only for one read.
            (value nil))
        (without-suspension
          (tagbody retry
             (let ((old-pulse-count (with-unsuspendable-lock value-lock pulse-count)))
               (or (setf value (value fluent))
                   ;; We need to do ugly explicit handling of suspension
                   ;; here.  WAIT-FOR is a very special case since we need
                   ;; to ensure that even if CONDITION-VARIABLE-WAIT gets
                   ;; suspended, the value-lock is released and we wait
                   ;; again after suspension.
                   (with-suspension
                     (handler-case
                         (with-lock-held (value-lock)
                           (unless (> pulse-count old-pulse-count)
                             (if timeout
                                 (condition-variable-wait-with-timeout changed-condition timeout)
                                 (condition-variable-wait changed-condition))))
                       (suspension (c)
                         (signal-suspension c)
                         t)))
                   (unless (or (and timeout (>= (/ (- (get-internal-real-time)
                                                      start-time)
                                                   internal-time-units-per-second)
                                                timeout))
                               (and (setf value (value fluent))
                                    (with-lock-held (value-lock)
                                      (> pulse-count old-pulse-count))))
                     (go retry))))))
        value))))

(defmethod pulse ((fluent fluent))
  (with-slots (changed-condition pulse-count on-update) fluent
    (without-termination
      (let ((on-update-list (with-fluent-locked fluent
                              (hash-table-values on-update)))
            (value nil))
        (with-fluent-locked fluent
          (setf value (peek-value fluent))
          (incf pulse-count)
          (condition-variable-broadcast changed-condition))
        (loop for callback in on-update-list
              do (funcall callback value))))))

(defmacro whenever ((condition-fluent &key (wait-status :waiting))
                    &body body)
  "Executes `body' whenever `condition-fluent' is pulsed or non-nil.
   The `body' forms are executed in an implicit block named NIL.  The
   parameter `wait-status' has the same meaning as in WAIT-FOR"
  (with-unique-names (body-fun)
    `(block nil
       (flet ((,body-fun ()
                ,@body))
         (let* ((fluent ,condition-fluent))
           (loop
              (wait-for fluent :wait-status ,wait-status)
              (,body-fun)))))))
