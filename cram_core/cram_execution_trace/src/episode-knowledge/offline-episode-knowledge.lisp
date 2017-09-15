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
;;; Offline episodes
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; NOTE ON THREAD SAFETY
;;;
;;; We guarantee no thread safety for offline episodes. We don't see why one
;;; would need to access this from multiple threads. If the use case arises,
;;; we can change it.

(defclass offline-episode-knowledge (episode-knowledge)
  ((max-time :reader max-time
             :type timestamp)
   (task-list :reader task-list
              :type list)
   (goal-task-list :reader goal-task-list
                   :type list)
   (fluent-changes-hash-table :type hash-table)
   (fluent-durations-hash-table :type hash-table)))

(defmethod initialize-instance :after ((episode offline-episode-knowledge)
                                       &key &allow-other-keys)
  (with-slots (execution-trace max-time zero-time task-tree task-list
               goal-task-list fluent-changes-hash-table fluent-durations-hash-table)
      episode
    (setf max-time (calculate-max-time execution-trace #'identity zero-time)
          task-list (calculate-task-list task-tree)
          goal-task-list (calculate-goal-task-list task-list)
          fluent-changes-hash-table (copy-hash-table execution-trace
                                                     :key #'calculate-fluent-changes)
          fluent-durations-hash-table (copy-hash-table fluent-changes-hash-table
                                                       :key (rcurry #'changes->durations
                                                                    max-time)))))

(defmethod task-tree ((episode offline-episode-knowledge))
  (slot-value episode 'task-tree))

(defmethod traced-fluent-instances ((episode offline-episode-knowledge) fluent-name)
  (with-slots (execution-trace) episode
    (gethash fluent-name execution-trace)))

(defmethod fluent-changes ((episode offline-episode-knowledge) fluent-name)
  (with-slots (fluent-changes-hash-table) episode
    (gethash fluent-name fluent-changes-hash-table)))

(defmethod fluent-durations ((episode offline-episode-knowledge) fluent-name)
  (with-slots (fluent-durations-hash-table) episode
    (gethash fluent-name fluent-durations-hash-table)))

(defmethod traced-fluents-hash-table ((episode offline-episode-knowledge))
  (slot-value episode 'execution-trace))

(defmethod traced-fluent-names ((episode offline-episode-knowledge))
  (with-slots (execution-trace) episode
    (hash-table-keys execution-trace)))

(defmethod running-p ((episode offline-episode-knowledge))
  (declare (ignore episode))
  nil)
