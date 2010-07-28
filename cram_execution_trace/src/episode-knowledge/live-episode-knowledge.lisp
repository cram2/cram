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
;;; Live episodes
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass live-episode-knowledge (episode-knowledge)
  ((running-flag :accessor running-flag :type boolean :initform nil 
                 :documentation "We use this to start logging at the beginning
                 of an epsiode and stop logging, when the epsiode is over. We
                 don't synchronize since we assume reading a CLOS slot to be
                 safe even if another thread simultaniously changes the
                 flag. We assume this to be thread safe.")
   (trace-queue :accessor trace-queue :type queue
                :documentation "One queue for all fluents to add their traced
                instances to. This needs to be a queue with thread safe
                access.")
   (fluents :accessor fluents :type hash-table
            :documentation "Weak hash table of all fluents that are created
            while this episode was running.")))

(defmethod initialize-instance :after ((episode live-episode-knowledge) &key new-task-tree)
  (with-slots (execution-trace fluents) episode
    (setf execution-trace (make-synchronized-hash-table :test 'eq))
    (setf fluents (make-synchronized-hash-table :test 'eq :weakness :key))
    (reset episode new-task-tree)))

(defmethod max-time ((episode live-episode-knowledge))
  "Compute the max time by searching all the traces."
  (with-slots (execution-trace zero-time) episode
    (with-hash-table-locked (execution-trace)
      (calculate-max-time execution-trace #'identity zero-time))))

(defmethod task-list ((episode live-episode-knowledge))
  (calculate-task-list (task-tree episode)))

(defmethod goal-task-list ((episode live-episode-knowledge))
  (calculate-goal-task-list (task-list episode)))

(defmethod fluent-trace-queue ((episode live-episode-knowledge) fluent-name)
  (declare (ignore fluent-name))
  (slot-value episode 'trace-queue))

(defun update-execution-trace (episode)
  "Helper function to dequeue all traced instances from the trace queue and
  put the in the execution trace. Should be called by all functions that use
  the execution trace to update it with new traced instances before processing
  it."
  (with-slots (trace-queue execution-trace) episode
    (let ((new-instances (make-hash-table)))
      ;; 1. Empty the queue
      (loop for instance = (dequeue trace-queue)
         ;; Using m-v-l here is a bit ugly and might cons (unless the compiler
         ;; is able to optimize), but seems to me the most concise portable
         ;; idiom for doing what i want.
         while instance
         do (push instance (gethash (name instance) new-instances)))
      ;; 2. Then append the new instances to the ones allready in execution-trace
      (loop for rev-inst-list being the hash-values in new-instances using (hash-key inst-name)
         do (setf (gethash inst-name execution-trace)
                  (append (gethash inst-name execution-trace)
                          (reverse rev-inst-list)))))))

(defmethod traced-fluent-instances ((episode live-episode-knowledge) fluent-name)
  (update-execution-trace episode)
  (with-slots (execution-trace) episode
    (multiple-value-bind (insts found?) (gethash fluent-name execution-trace)
      (if found?
          (values insts t)
          (values nil nil)))))

(defmethod traced-fluents-hash-table ((episode live-episode-knowledge))
  (update-execution-trace episode)
  (with-slots (execution-trace) episode
    (with-hash-table-locked (execution-trace) 
      (copy-hash-table execution-trace))))

(defmethod traced-fluent-names ((episode live-episode-knowledge))
  (update-execution-trace episode)
  (with-slots (execution-trace) episode
    (with-hash-table-locked (execution-trace)
      (hash-table-keys execution-trace))))

(defmethod fluent-changes ((episode live-episode-knowledge) fluent-name)
  (calculate-fluent-changes (traced-fluent-instances episode fluent-name)))

(defmethod fluent-durations ((episode live-episode-knowledge) fluent-name)
  (changes->durations (fluent-changes episode fluent-name)
                      (max-time episode)))

(defun reset (live-episode new-task-tree)
  (with-slots (execution-trace task-tree zero-time
               end-time running-flag trace-queue fluents)
      live-episode
    (clrhash execution-trace)
    (clrhash fluents)
    (setf zero-time       nil
          end-time        nil
          running-flag    nil
          trace-queue     (make-queue :name "Trace queue")
          task-tree       new-task-tree)))

(defun start (live-episode)
  (setf (zero-time live-episode) (current-timestamp)
        (running-flag live-episode) t))

(defun stop (live-episode)
  (setf (running-flag live-episode) nil)
  (setf (end-time live-episode) (current-timestamp)))

(defmethod running-p ((episode live-episode-knowledge))
  (and episode (running-flag episode)))

(defun unregister-fluent-callbacks (episode)
  (loop for fluent being the hash-keys in (fluents episode)
     do (remove-update-callback fluent :fluent-tracing-callback)))

(defun register-episode-fluent (fluent episode)
  (setf (gethash fluent (fluents episode)) t))

;; TODO: Evaluate if we need to export reset, stop, start etc since we use
;; hooks now. Also maybe dont export the top-level-episode-knowledge access
;; (maybe do though, for introspection)

(defmethod on-top-level-setup-hook :execution-trace (top-level-name task-tree)
  (let ((ek (if top-level-name
                (get-top-level-episode-knowledge top-level-name)
                (make-instance 'live-episode-knowledge))))
    (reset ek task-tree)
    (start ek)
    (let ((*episode-knowledge* ek))
      (trace-global-fluents))
    (cons (list '*episode-knowledge*) (list ek))))

(defmethod on-top-level-cleanup-hook :execution-trace (top-level-name)
  (stop *episode-knowledge*)
  (unregister-fluent-callbacks *episode-knowledge*)
  (setf *last-episode-knowledge* *episode-knowledge*)
  (when (auto-tracing-enabled)
    (save-episode *episode-knowledge* (auto-tracing-filepath top-level-name))))
