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

(in-package #:cram-execution-trace)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Base class
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define-task-variable *episode-knowledge* nil
  "The episode knowledge of the current episode. This will be bound within a
   cram top level using the hooks provided there.")

(defvar *last-episode-knowledge* nil
  "This always contains the episode-knowledge object of the episode that was
   executed (and finished) last. This is used for easy interactive access.")

(defclass episode-knowledge ()
  ((zero-time :accessor zero-time :initarg :zero-time :type (or null timestamp)
              :documentation "Start time of episode.")
   (end-time  :accessor end-time  :initarg :end-time :type (or null timestamp)
              :documentation "End time of episode.")
   (task-tree :initarg :task-tree :type (or null task-tree-node))
   (execution-trace :initarg :execution-trace :type hash-table
                    :documentation "Hash table with all traced instances
                    indexed by fluet name.")))

;; Difference between end-time and max-time is, that end-time should be set by
;; top-level when it finishes and max-time is calculated by looking at the
;; most recent traced fluent change.

(defgeneric max-time (episode-knowledge))

(defgeneric task-tree (episode-knowledge)
  (:documentation "Returns the task tree. (The tree will be returned without
  stale nodes and without the dummy root with no path, i.e. the top-level node
  will be the root node.)"))

(defgeneric task-list (episode-knowledge))

(defgeneric goal-task-list (episode-knowledge))

(defgeneric fluent-trace-queue (episode-knowledge fluent-name))

(defgeneric traced-fluent-instances (episode-knowledge fluent-name))

(defgeneric traced-fluents-hash-table (episode-knowledge))

(defgeneric traced-fluent-names (episode-knowledge))

(defgeneric fluent-changes (episode-knowledge fluent-name))

(defgeneric fluent-durations (episode-knowledge fluent-name))

(defgeneric running-p (episode-knowledge))
