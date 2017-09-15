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

(defun trace-fluent (fluent value episode)
  "Trace the current value of `fluent'."
  ;; Enqueuing must be thread safe, since multiple threads trace fluents at
  ;; the same time.
  (when (and (fluent-tracing-enabled) (running-p episode))
    (enqueue (make-instance 'traced-fluent
               :name (name fluent)
               :traced-value (durable-copy value))
             (fluent-trace-queue episode fluent))))

(defun trace-fluent-callback (fluent max-tracing-freq episode)
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
              (trace-fluent fluent value episode)))))
      (lambda (value) (trace-fluent fluent value episode))))

;;;
;;; ABOUT REGISTER-FLUENTS
;;;
;;; We keep all fluents created in a hash table. If a fluent is created while
;;; an episode is running, it is stored in the episodes hash table, otherwise
;;; in a global hash table.
;;;
;;; The update-callback that does the tracing for each fluent is a closure
;;; over the episode-knowledge object it should trace to. It does not use the
;;; *episode-knowledge* variable, since it has task-local bindings only. If a
;;; fluent is pulsed by a thread that with no binding for *episode-knowledge*
;;; (i.e. a non-task thread), tracing would not work if we relied on
;;; *epiosde-knowledge* being set. (Of course we could just never rebind
;;; *episode-knowledge* and setf the one global binding, but I don't want to
;;; do that for other reasons).
;;;
;;; For "episode-local" fluents the callback is registered immediately after
;;; creation, since the episode is allready running then. For all global
;;; fluents the callbacks are registered whenever an episode is started. Note
;;; that we need to immediately trace the (initial) value after registering
;;; the callback, since a fluent might never pulse during the course of an
;;; episode, but it still has a value.
;;;
;;; All fluents get their fluent tracing callback unregistered when their
;;; episode stops. This is to avoid them being traced in the next episode (a
;;; fluent-net might be pulsed by a global fluent like the robot position or
;;; other sensor data, even after the episode has ended).
;;;

(defvar *global-fluents* (make-synchronized-hash-table :test 'eq :weakness :key))

(defun register-fluent (fluent options)
  (if (episode-running)
      (register-episode-fluent fluent *episode-knowledge*)
      (register-global-fluent fluent options)))

(defun register-global-fluent (fluent options)
  (setf (gethash fluent *global-fluents*) options))

(defun global-fluents-register-callbacks (episode)
  (loop for fluent being the hash-keys in *global-fluents* using (hash-value options)
     for max-tracing-freq = (getf options :max-tracing-freq)
        unless (get-update-callback fluent :fluent-tracing-callback)
          do (register-update-callback fluent :fluent-tracing-callback
                                              (trace-fluent-callback fluent max-tracing-freq episode))
             (trace-fluent fluent (peek-value fluent) episode)))

(defun global-fluents-unregister-callbacks ()
  (loop for fluent being the hash-keys in *global-fluents*
     do (remove-update-callback fluent :fluent-tracing-callback)))

(defmethod on-make-fluent-hook :execution-trace (fluent allow-tracing max-tracing-freq)
  (when allow-tracing
    ;; NOTE: Possible race condition if other thread (1) setfs
    ;; *episode-knowledge* or the (2) running status changes within the
    ;; following couple of commands. However neither might actually occour in
    ;; practice. (1) prob. can not happen since we use thread local bindings
    ;; for *episode-knowledge*. (2) prob. can not happen since the episode
    ;; stops only when all tasks have finished, meaning their threads have
    ;; stopped as well.
    (register-fluent fluent `(:max-tracing-freq ,max-tracing-freq))
    (when (episode-running)
      (register-update-callback fluent :fluent-tracing-callback
                                (trace-fluent-callback fluent max-tracing-freq
                                                       *episode-knowledge*))
      (trace-fluent fluent (peek-value fluent) *episode-knowledge*))))
