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

;;;;; TODO: REVIEW especially thread safety of execution trace access.

;;; NOTE ON THREAD SAFETY
;;;
;;; For live episode knowledge we make some limited guarantees on thread
;;; safety.
;;;
;;; 1. We assume that the running-flag need not be protected by a lock. The
;;; operations STRAT, STOP and RUNNING-P should not interfere since all they
;;; do is either read oder setf the slot which we assume to be atomic. The
;;; worst that could happen is that a fluent wants to add a trace "at the same
;;; time" as STOP is called. So the trace might be added to the queue even
;;; though the episode is not strictly running any more. This should be
;;; fine. At the moment UNREGISTER-FLUENT-CALLBACKS is called straight after
;;; STOP. UNREGISTER-FLUENT-CALLBACKS implicitely has to aquire each fluent's
;;; value lock, so after UNREGISTER-FLUENT-CALLBACKS has returned no more
;;; stray traces should come in.
;;;
;;; 2. We assume that there is no parallel access to the episode-knowledge
;;; before START and after STOP (except for checking the running flag).
;;;
;;; 3. Appending traced instances is synchronized by using a thread safe
;;; queue.
;;;
;;; 4. Access to the execution trace is synchronized by using a synchronized
;;; hash table and locking in the apporpriate places.
;;;
;;; 5. Access to the task tree is not synchronized and may not be thread safe.
;;; 

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
            :documentation "Weak, synchronized hash table of all fluents that
            are created while this episode was running.")
   (filtered-task-tree :reader task-tree :type task-tree-node
                       :documentation "The filtered task tree. We need
                       to cache that value since filtering creates new
                       task objects and we need to assure equality of
                       tasks with the same path.")))

(defmethod initialize-instance :after ((episode live-episode-knowledge) &key new-task-tree)
  (with-slots (execution-trace fluents) episode
    (setf execution-trace (make-synchronized-hash-table :test 'eq))
    (setf fluents (make-synchronized-hash-table :test 'eq :weakness :key))
    (reset episode new-task-tree)))

(defmethod max-time ((episode live-episode-knowledge))
  "Compute the max time by searching all the traces."
  (update-execution-trace episode)
  (with-slots (execution-trace zero-time) episode
    (with-hash-table-locked (execution-trace)
      (calculate-max-time execution-trace #'identity zero-time))))

(defmethod task-tree :before ((episode live-episode-knowledge))
  (unless (slot-boundp episode 'filtered-task-tree)
    (let ((top-level (cdar (task-tree-node-children (slot-value episode 'task-tree)))))
      (unless top-level
        (error "Task tree has no top level node."))
      (setf (slot-value episode 'filtered-task-tree)
            (filter-task-tree (compose #'not #'stale-task-tree-node-p)
                              top-level)))))

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
      ;; 1. Empty the queue into a temporary hash table
      (loop for instance = (dequeue trace-queue)
         while instance
         do (push instance (gethash (name instance) new-instances)))
      ;; 2. Then append the new instances to the ones allready in execution-trace
      (with-hash-table-locked (execution-trace)
        (loop for rev-inst-list being the hash-values in new-instances using (hash-key inst-name)
           do (setf (gethash inst-name execution-trace)
                    (append (gethash inst-name execution-trace)
                            (reverse rev-inst-list))))))))

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
          task-tree       new-task-tree)
    (slot-makunbound live-episode 'filtered-task-tree)))

(defun start (live-episode)
  (setf (zero-time live-episode) (current-timestamp)
        (running-flag live-episode) t))

(defun stop (live-episode)
  (setf (running-flag live-episode) nil)
  (setf (end-time live-episode) (current-timestamp)))

(defmethod running-p ((episode live-episode-knowledge))
  (running-flag episode))

(defun unregister-fluent-callbacks (episode)
  (with-hash-table-locked ((fluents episode))
    (loop for fluent being the hash-keys in (fluents episode)
       do (remove-update-callback fluent :fluent-tracing-callback))))

(defun register-episode-fluent (fluent episode)
  (setf (gethash fluent (fluents episode)) t))

(defmethod on-top-level-setup-hook :execution-trace (top-level-name task-tree)
  (let ((ek (make-instance 'live-episode-knowledge)))
    (reset ek task-tree)
    (start ek)
    (global-fluents-register-callbacks ek)
    (cons (list '*episode-knowledge*) (list ek))))

(defmethod on-top-level-cleanup-hook :execution-trace (top-level-name)
  (stop *episode-knowledge*)
  (unregister-fluent-callbacks *episode-knowledge*)
  (global-fluents-unregister-callbacks)
  (setf *last-episode-knowledge* *episode-knowledge*)
  (set-top-level-episode-knowledge top-level-name *episode-knowledge*)
  (when (auto-tracing-enabled)
    (save-episode *episode-knowledge* (auto-tracing-filepath top-level-name))))
