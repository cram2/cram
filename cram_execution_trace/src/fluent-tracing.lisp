;;;
;;; Copyright (c) 2009 - 2010
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

(in-package :cram-execution-trace)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Traced instance
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; NOTE 30.04.2010 by @demmeln: As of now only fluents are traced, so
;;; TRACED-INSTANCE as a seperate base class is not really neccessary. We
;;; leave it in for now anyway for possible future extensions.

(defclass traced-instance ()
  ((timestamp :initform (current-timestamp) :initarg :timestamp :reader timestamp))
  (:documentation "Traced instances are the objects actually saved in the
                   trace. All traces have a timestamp."))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Fluent tracing
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(let ((tracing-enabled nil))
  (defun enable-fluent-tracing ()
    (setf tracing-enabled t))
  (defun disable-fluent-tracing ()
    (setf tracing-enabled nil))
  (defun fluent-tracing-enabled ()
    tracing-enabled))

(defclass traced-fluent (traced-instance)
  ((name :initarg :name :reader name)
   (traced-value :initarg :traced-value :reader traced-value)))

(defun trace-fluent (fluent value)
  "Trace the current value of `fluent'."
  ;; Enqueuing must be thread safe, since multiple threads trace fluents at
  ;; the same time.
  (when (and (fluent-tracing-enabled) (episode-running))
    (enqueue (make-instance 'traced-fluent
               :name (name fluent)
               :traced-value (durable-copy value))
             (fluent-trace-queue *episode-knowledge* fluent))))

(defun trace-fluent-callback (fluent max-tracing-freq)
  "Returns a closure to be registerd as a on-update callback for fluents. If
   `max-tracing-freq' is not NIL it will be used to make sure that the
   frequency with which updates to this fluent are traced does not exceed the
   value in `max-tracing-freq'. This can be useful for frequently updating
   fluents."
  (if max-tracing-freq
      (let* ((min-time-between-traces (/ 1 max-tracing-freq))
             ;; make sure the fluent is traced the first time the callback is
             ;; called
             (last-time (- (current-timestamp)
                           (* 2 min-time-between-traces))))
        (lambda (value)
          (let ((current-time (current-timestamp)))
            ;; only trace if enough time has ellapsed
            (when (<= min-time-between-traces (- current-time last-time))
              (setf last-time current-time)
              (trace-fluent fluent value)))))
      (lambda (value) (trace-fluent fluent value))))

;;; TODO, FIXME: We need to ensure, that the initial value of fluents is
;;; traced when an episode starts. For this we 1) need to trace it upon
;;; creation (in on-make-fluent-hook) and 2) need to trace it when an epsiode
;;; starts (in case the fluent was created prior to the epsiode start). For
;;; this to work we need to register all fluents in a global weak hash-table
;;; to be able to trigger tracing.

(defmethod on-make-fluent-hook :execution-trace (fluent allow-tracing max-tracing-freq)
  (when allow-tracing
    (register-update-callback fluent :fluent-tracing-callback
                              (trace-fluent-callback fluent max-tracing-freq))
    (trace-fluent fluent (peek-value fluent))))
