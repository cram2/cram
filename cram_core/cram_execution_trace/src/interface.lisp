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

;;; TODO: Beware that accessing the execution trace of a live episode is not
;;; neccesarily thread safe any more... check this out.

;;; TODO: Update these notes:

;;; The epsiode knowedge consists of a task tree and an execution trace. There
;;; are two kinds of episode knowledge. LIVE-EPISODE-KNOWLEDGE and
;;; OFFLINE-EPISODE-KNOWLEDGE both deriving from EPISODE-KNOWLEDGE.
;;; 
;;; LIVE-EPISODE-KNOWLEDGE supports adding new (fluent-) traces to the
;;; execution trace and its TASK-TREE references real TASK objects. It might
;;; be, that some tasks are still executed. Every TOP-LEVEL(-PLAN) is
;;; associated with its own LIVE-EPIOSDE-KNOWLEDGE, which will be reset every
;;; time the plan is executed.
;;;
;;; OFFLINE-EPISODE-KNOWLEDGE has been loaded (from a file), doesn't support
;;; additional traces being added and its TASK-TREE references OFFLINE-TASK
;;; objects which contain only the correct fluent names and result values (but
;;; e.g. no thread objects).
;;;
;;; Both kinds of EPISODE-KNOWLEDGE objects can be queried for their (current)
;;; execution-trace / task-tree and they can be saved to a file.
;;;
;;; Accessing the execution trace of a live episode is thread safe, but
;;; accessing the task tree might not be. The offline episode is not thread
;;; safe at all (but it doesn't need to be anyway).

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Interface
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *top-level-episode-knowledge-lock* (sb-thread:make-mutex))

(defvar *top-level-episode-knowledge* (make-hash-table :test 'eq)
  "The episode knowledge for named top-levels (top level plans) is stored in
   this hash-table indexed by the plan name (which is a symbol).")

(defun register-top-level-episode-knowledge (name)
  "Registers a live-episode-knowledge object for `name'. This is done
   automatically for top-level plans. If you want to use named-top-levels with
   tracing manually, you need to register the name manually."
  (sb-thread:with-mutex (*top-level-episode-knowledge-lock*)
    (setf (gethash name *top-level-episode-knowledge*)
          (make-instance 'live-episode-knowledge))))

(defun get-top-level-episode-knowledge (name)
  "Returns the episode-knowledge of the top level plan.

   We use this, so named top levels (e.g. the ones used in top level plans),
   can use the same episode-knowledge object over and over again and more
   importantly, you can easily access it with this function after the plan has
   run."
  (declare (type symbol name))
  (or (sb-thread:with-mutex (*top-level-episode-knowledge-lock*)
        (gethash name *top-level-episode-knowledge*))
      (error "Accessing top-level episode-knowledge for unregisterd top-level plan ~a."
             name)))

(defun set-top-level-episode-knowledge (name episode-knowledge)
  (declare (type symbol name)
           (type episode-knowledge episode-knowledge))
  (sb-thread:with-mutex (*top-level-episode-knowledge-lock*)
    (setf (gethash name *top-level-episode-knowledge*) episode-knowledge)))

(defun episode-knowledge-zero-time (&optional (episode *episode-knowledge*))
  "Starting time of episode. All timestampes (e.g. in traces) can be
   interpreted relative to zero time."
  (zero-time episode))

(defun episode-knowledge-max-time (&optional (episode *episode-knowledge*))
  "This is when the last trace was recorded. Notice how this is different from
   EPISODE-KNOWLEDGE-END-TIME."
  (max-time episode))

(defun episode-knowledge-end-time (&optional (episode *episode-knowledge*))
  "Ending time of the epsiode. Compute the timespan of the episode by
   subtracting the zero time. Notice how this is different from
   EPISODE-KNOWLEDGE-MAX-TIME."
  (end-time episode))

(defun episode-knowledge-task-tree (&optional (episode *episode-knowledge*))
  "The task tree for this episode."
  (task-tree episode))

(defun episode-knowledge-task-list (&optional (episode *episode-knowledge*))
  "A list of all tasks in the task tree."
  (task-list episode))

(defun episode-knowledge-goal-task-list (&optional (episode *episode-knowledge*))
  "A list of all goal tasks in the task tree."
  (goal-task-list episode))

(defun episode-knowledge-traced-fluent-names (&optional (episode *episode-knowledge*))
  "Returns a list of names of all traced fluents."
  (traced-fluent-names episode))

(defun episode-knowledge-fluent-changes
    (fluent-name &optional (episode *episode-knowledge*))
  "Returns a list of cons cells - one cons for each time the fluents value
   changed - with the car being the value and cdr being the timestamp."
  (fluent-changes episode fluent-name))

(defun episode-knowledge-fluent-durations
    (fluent-name &optional (episode *episode-knowledge*))
  (fluent-durations episode fluent-name))

(defun episode-running ()
  "Returns T if an episode is currently running. NIL otherwise."
  (and *episode-knowledge* (running-p *episode-knowledge*)))

(defmacro with-episode-knowledge (episode-knowledge &body body)
  `(let ((*episode-knowledge* ,episode-knowledge))
     ,@body))

(defmacro with-last-episode-knowledge (&body body)
  `(with-episode-knowledge *last-episode-knowledge*
     ,@body))

(defmacro with-top-level-episode-knowledge (plan-name &body body)
  `(with-episode-knowledge (get-top-level-episode-knowledge ',plan-name)
     ,@body))

(defmacro with-offline-episode-knowledge (source &body body)
  `(with-episode-knowledge (load-episode-knowledge ,source)
     ,@body))

(defun save-episode-knowledge (destination
                               &optional (episode *episode-knowledge*)
                               &rest key-args)
  "Saves an episode to a file or stream. You can pass a keyword argument
   `if-exists' which is passed to OPEN if `destination' is a string or a
   pathname. Default for if-exists is :ERROR."
  (when (eq episode :if-exists)
    (push :if-exists key-args)
    (setf episode *episode-knowledge*))
  (apply #'save-episode episode destination key-args))

(defun save-last-episode-knowledge (destination &key (if-exists :error))
  "Saves the episode-knowledge of the epsiode run last to `destination'. See
   SAVE-EPISODE-KNOWLEDGE."
  (with-last-episode-knowledge
    (save-episode-knowledge destination :if-exists if-exists)))

(defun load-episode-knowledge (source)
  "Loads an episode from a file or stream. Returns an OFFLINE-EPISODE-KNOWLDGE
   object."
  (load-episode source))
