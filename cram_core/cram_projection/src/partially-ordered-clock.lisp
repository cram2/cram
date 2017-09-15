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

(defclass partially-ordered-clock ()
  ((time :initarg :initial-time :initform 0 :reader clock-time)
   (increment-delay :initarg :increment-delay :initform 0.05
                    :reader increment-delay
                    :documentation "Delay (in seconds) before the next
                    time step can be performed. When CLOCK-WAIT has
                    been called,`time' can only be incremented after
                    `increment-delay' expired. Please note that a
                    value that is too small (depending on the
                    scheduler and CPU load) will result in behavior
                    similar to LINEAR-CLOCK.")
   (disabled :initform nil)
   (lock :initform (sb-thread:make-mutex))
   (condition :initform (sb-thread:make-waitqueue))
   (wait-queue :initform nil)))

(defgeneric partially-ordered-clock-enabled (clock)
  (:documentation "Returns if the clock is enabled at the moment.")
  (:method ((clock partially-ordered-clock))
    (with-slots (lock disabled) clock
      (sb-thread:with-mutex (lock) clock
        (not disabled)))))

(defun enqueue-duration (clock name duration)
  (with-slots (lock time wait-queue) clock
    (sb-thread:with-mutex (lock)
      (setf wait-queue
            (sort (cons (cons name (+ time duration)) wait-queue)
                  #'< :key #'cdr)))))

(defun dequeue-duration (clock name)
  (with-slots (lock wait-queue) clock
    (sb-thread:with-mutex (lock)
      (setf wait-queue (delete name wait-queue :key #'car)))))

(defun duration-expired (clock name)
  (with-slots (lock time wait-queue) clock
    (sb-thread:with-mutex (lock)
      (let ((expiration-time (cdr (assoc name wait-queue))))
        (assert expiration-time)
        (<= expiration-time time)))))

(defun enqueue-disabling-process (clock name)
  (with-slots (lock condition disabled) clock
    (sb-thread:with-mutex (lock)
      (push name disabled)
      (sb-thread:condition-broadcast condition))))

(defun dequeue-disabling-process (clock name)
  (with-slots (lock condition disabled) clock
    (sb-thread:with-mutex (lock)
      (setf disabled (delete name disabled))
      (sb-thread:condition-broadcast condition))))

(defmacro with-partially-ordered-clock-disabled (clock &body body)
  (let ((disabled-symbol (gensym "CLOCK-DISABLED")))
    `(unwind-protect
          (progn
            (enqueue-disabling-process ,clock ',disabled-symbol)
            ,@body)
       (dequeue-disabling-process ,clock ',disabled-symbol))))

(defun wait-for-enabled (clock)
  "Waits for the clock to be enabled."
  (with-slots (lock disabled condition) clock
    (flet ((wait-for-enabled-with-mutex ()
             (loop while disabled
                   do (sb-thread:condition-wait condition lock))))
      ;; Special handling if we already hold the mutex: in order to
      ;; guarantee atomicity, we allow the caller to hold the mutex
      ;; already. In that case we don't acquire it.
      (if (sb-thread:holding-mutex-p lock)
          (wait-for-enabled-with-mutex)
          (sb-thread:with-mutex (lock)
            (wait-for-enabled-with-mutex))))))

(defun step-clock (clock)
  (with-slots (time lock wait-queue increment-delay)
      clock
    ;; Waiting for enabled twice here seems weird but we want to sleep
    ;; only after we are actually enabled. However, since we cannot
    ;; release the lock, sleep and re-acquire it as condition-wait
    ;; does it, we cannot do the wait-for while holding the lock. But
    ;; we also must not update the time when being disabled. That's
    ;; why we first wait for enabled, sleep, wait again, this time
    ;; inside the lock and then update the time.
    (wait-for-enabled clock)    
    (cpl:sleep increment-delay)
    (sb-thread:with-mutex (lock)
      (wait-for-enabled clock)
      (let ((next-expiration-time (cdr
                                   (find-if (lambda (wait-information)
                                              (> (cdr wait-information) time))
                                            wait-queue))))
        (when next-expiration-time
          (assert (>= next-expiration-time time))
          (setf time next-expiration-time))))))

(defmethod clock-wait ((clock partially-ordered-clock) duration)
  (declare (type number duration))
  (let ((wait-id (gensym "CLOCK-WAIT-")))
    (enqueue-duration clock wait-id duration)
    (unwind-protect
         (loop until (duration-expired clock wait-id)
               do (step-clock clock))
      (dequeue-duration clock wait-id))))
