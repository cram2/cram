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

(in-package :cl-user)

(defpackage :cram-execution-trace
  (:nicknames :cet)
  (:documentation
   "The execution trace records plan execution to later analyse what happened
    during plan execution.")
  (:use :common-lisp
        :cl-store
        :alexandria
        :cram-utilities
        :cram-language-implementation)
  (:export
   ;; durable-copy.lisp
   #:durable-copy
   ;; episode-knwoledge.lisp
   #:throughout
   #:episode-knowledge
   #:live-episode-knowledge
   #:offline-episode-knowledge
   #:*episode-knowledge*
   #:*last-episode-knowledge*
   #:get-top-level-episode-knowledge
   #:register-top-level-episode-knowledge
   #:defstore-episode-knowledge-backend
   #:defrestore-episode-knowledge-backend
   #:episode-knowledge-zero-time
   #:episode-knowledge-max-time
   #:episode-knowledge-task-tree
   #:episode-knowledge-task-list
   #:episode-knowledge-goal-task-list
   #:episode-knowledge-traced-fluent-names
   #:episode-knowledge-fluent-changes
   #:episode-knowledge-fluent-durations
   #:save-episode-knowledge
   #:save-last-episode-knowledge
   #:load-episode-knowledge
   #:with-episode-knowledge
   #:with-last-episode-knowledge
   #:with-top-level-episode-knowledge
   #:with-offline-episode-knowledge
   ;; tracing-fluent.lisp
   #:enable-fluent-tracing
   #:disable-fluent-tracing
   #:fluent-tracing-enabled
   #:traced-fluent
   #:timestamp
   #:name
   #:traced-value
   ;; auto-tracing.lisp
   #:enable-auto-tracing
   #:disable-auto-tracing
   #:auto-tracing-enabled
   #:set-auto-tracing-directory
   #:setup-auto-tracing))
