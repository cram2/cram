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

(defmacro with-fluent-locked (fluent &body body)
  `(with-lock-held ((slot-value ,fluent 'value-lock))
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

(defmethod print-object ((fluent fluent) stream)
  (print-unreadable-object (fluent stream :type nil :identity nil)
    (format stream "~@<FLUENT ~A~:>" (name fluent))))

(defmethod register-update-callback ((fluent fluent) name update-fun)
  (with-fluent-locked fluent
    (if (gethash name (slot-value fluent 'on-update))
        (error "Callback with name ~a is already registered." name)
        (setf (gethash name (slot-value fluent 'on-update)) update-fun))))

(defmethod get-update-callback ((fluent fluent) name)
  (with-fluent-locked fluent
    (gethash name (slot-value fluent 'on-update))))

(defmethod remove-update-callback ((fluent fluent) name)
  (with-fluent-locked fluent
    (remhash name (slot-value fluent 'on-update))))

(defmethod wait-for ((fluent fluent) &key (timeout nil))
  (log-event
    (:context "WAIT-FOR")
    (:display "~S~@[ ~:_:TIMEOUT ~S~]" fluent timeout)
    (:tags :wait-for))
  (if timeout
      (%timed-wait-for fluent timeout)
      (let ((condition-var (slot-value fluent 'changed-condition))
            (value-lock    (slot-value fluent 'value-lock)))
        (retry-after-suspension
         (with-lock-held (value-lock)
           (loop when (value fluent) return it
                 else do (condition-variable-wait condition-var)))))))

(declaim (inline recalculate-timeout))
(defun recalculate-timeout (timeout start-time)
  (- timeout (/ (float (- (get-internal-real-time) start-time) 1d0)
                (float internal-time-units-per-second 1d0))))

(defun %timed-wait-for (fluent timeout)
  (declare (fluent fluent))
  (let ((condition-var (slot-value fluent 'changed-condition))
        (value-lock    (slot-value fluent 'value-lock))
        (start-time    (get-internal-real-time)))
    (retry-after-suspension
      (setq timeout (recalculate-timeout timeout start-time))
      (with-lock-held (value-lock)
        (loop when (value fluent)        return it
              when (not (plusp timeout)) return nil
              else do
                (condition-variable-wait-with-timeout condition-var timeout)
                (setq timeout (recalculate-timeout timeout start-time)))))))

(defmethod pulse ((fluent fluent))
  (with-slots (changed-condition pulse-count on-update) fluent
    (without-scheduling
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
