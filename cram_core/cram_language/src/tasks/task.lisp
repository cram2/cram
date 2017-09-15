;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>
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

;;; FIXME: these should be moved to task-tree.lisp.
;;;        or perhaps above TOP-LEVEL in base.lisp.
;;;        with the DEFINE-TASK-VARIABLE supporting :INIT-FUNCTION
;;;        I think /both/ of them could possibly be moved there.

(define-task-variable *task-tree* nil
  "The task tree of the current top-level plan.")

(define-task-variable *current-task-tree-node* nil
  "The current task tree node.")

(defvar *synchronous-events* t "Indicates if we want to use synchronized events")

(defclass abstract-task ()
  ((name
    :reader task-name
    :reader name
    :initarg :name
    :initform (gensym "UNKNOWN-")
    :type (or symbol string null)
    :documentation "The name of the task. Mostly for debugging reasons.
                    Should also become the name of the thread executing
                    the task.")
   (thread
    :initform nil
    :type (or thread null)
    :documentation "The internal thread object of the lisp implementation.")
   (parent-task
    :initform nil
    :reader parent-task
    :type (or abstract-task null)
    :documentation "The parent task.")
   (child-tasks
    :initform (list)
    :accessor child-tasks
    :type (list-of abstract-task) 
    :documentation "The list of child tasks.")
   (status                              ; initialized below
    :reader status
    :type fluent
    :documentation "Fluent indicating the task status. 
                    Cf. STATUS-INDICATOR.")
   (result
    :initform nil
    :type (or list null condition)
    :documentation "The result of the task. When it terminates
		    normally, the slot contains the list of return
		    values. For evaporated tasks, it contains nil and
		    for failed tasks, it contains the condition
		    object.")
   (thread-fun
    :initform nil
    :initarg :thread-fun
    :type (or function null)
    :documentation "The function of the task.")
   (path
    :initform *current-path*
    :initarg :path
    :reader task-path
    :type list
    :documentation "The path of the task.")
   (code
    :initform (and *current-task-tree-node*
                   (task-tree-node-code *current-task-tree-node*))
    :reader task-code
    :type (or code null)
    :documentation "The code of the path.")
   (constraints
    :initform (list)
    :accessor task-constraints
    :type (list-of fluent)
    :documentation "The list of ordering constraint fluents. The task
		    is not allowed to start running before all fluents
		    are true.")
   (message-queue
    :reader task-message-queue
    :initform (make-mailbox)
    :type mailbox)))

(defclass toplevel-task (abstract-task)
  ())

(defclass task (abstract-task)
  ())

(defmethod initialize-instance :after ((task abstract-task) &key (run-thread t))
  (with-slots (name status thread-fun) task
    (setf status (make-fluent :name (format-gensym "~A-STATUS" name)
                              :value :created))
    (when (and thread-fun run-thread)
      (execute task))))

(defmethod print-object ((task abstract-task) stream)
  (let ((level *task-pprint-verbosity*))
    (print-unreadable-object (task stream :type t :identity t)
      (format stream "~S" (string (task-name task)))
      ;; We depend on the verbosity level here because VALUE goes over
      ;; a lock.
      (when (> level 0)
        (format stream " ~S" (value (status task)))))))

(defun make-dummy-task (thread)
  (declare (type thread thread))
  (let ((dummy (make-instance 'toplevel-task :name (thread-name thread))))
    (setf (slot-value dummy 'thread) thread)
    (setf (value (status dummy)) :running)
    dummy))

(defun task-alive (task)
  "Returns a fluent indicating if the task is alive"
  (fl-funcall #'member (status task) +alive+))

(defun task-dead (task)
  (fl-funcall #'member (status task) +dead+))

(defun task-running-p (task)
  (member (value (status task)) +alive+))

(defun task-dead-p (task)
  (member (value (status task)) +dead+))

(defun task-done-p (task)
  (member (value (status task)) +done+))

(defun task-failed-p (task)
  (eq (value (status task)) :failed))

(defun assert-status (task status &rest more)
  (let ((actual-status (value (status task)))
        (expected (cons status more)))
    (assert (member actual-status expected) ()
            "~@<~S has status ~S, ~:_expected ~:[~S~;one of ~{~S~^, ~}~]~:>"
            task actual-status more (if more expected status))))

(defmethod executed ((task abstract-task))
  (not (eq (value (status task)) :created)))

(defmethod result ((task abstract-task))
  (with-slots (result) task
    (if (listp result)
        (values-list result)
        result)))

(defmethod join-task ((task abstract-task))
  (when (eq task *current-task*)
    (error "Cannot join current task."))
  (log-event
    (:context "JOIN-TASK")
    (:display "~A" (task-abbreviated-name task))
    (:tags :join-task))
  (wait-for (task-dead task))
  (if (task-failed-p task)
      (assert-no-returning
        (signal (result task)))
      (result task)))

(eval-when (:compile-toplevel :load-toplevel :execute)
    (defparameter *status-transitions*
    '((:created    -> :running :evaporated)
      (:running    -> :suspended :succeeded :failed :evaporated)
      (:suspended  -> :running :evaporated :failed)
      (:succeeded  -> )
      (:failed     -> )
      (:evaporated -> ))
    "Valid transitions from one task's status to the next."))

(defun change-status (task new-status)
  (macrolet 
      ((check-status-transition (old new)
         `(ecase ,old 
            ,@(loop for (from -> . tos) in *status-transitions*
                collect `((,from)
                          (assert (member ,new ',tos)
                                  (,new)
                                  "Invalid status transition from ~S to ~S."
                                  ,old ,new))))))
    (let ((old-status (value (status task))))
      (check-status-transition old-status new-status)
      (setf (value (status task)) new-status)
      (log-event
        (:context "CHANGE-STATUS")
        (:display "~S -> ~S" old-status new-status)
        (:tags :change-status)))))

(defmethod suspend ((task abstract-task) &key reason (sync *synchronous-events*))
  (send-event task (make-event :suspend reason sync)))

(defmethod wake-up ((task abstract-task) &key reason (sync *synchronous-events*))
  (send-event task (make-event :wakeup reason sync)))

(defmethod evaporate ((task abstract-task) &key reason (sync *synchronous-events*))
  (send-event task (make-event :evaporate reason sync)))


(defun setup (child-task parent-task)
  "Initialize `child-task' for execution.
   Also register it as child to `parent-task'."
  (cond ((typep child-task 'toplevel-task)
         (assert (null parent-task)))
        (t
         (assert (eq parent-task *current-task*))
         ;; Make PARENT-TASK the parent of CHILD-TASK.
         (setf (slot-value child-task 'parent-task) parent-task)
         ;; Make CHILD-TASK a child of PARENT-TASK.
         (with-slots (child-tasks) parent-task
           (assert (not (member child-task child-tasks)) ()
                   "Attempt to register ~S twice to ~S." child-task parent-task)
           (push child-task child-tasks))
         (log-event
           (:context "REGISTER-CHILD")
           (:display "~A" (task-abbreviated-name child-task))
           (:tags :register-child)))))

(defmethod execute :before ((task abstract-task))
  (assert (slot-value task 'thread-fun) ()
          "Tried to execute task ~S which has no thread-fun." task)
  (assert (not (slot-value task 'thread)) ()
          "Task ~S is already running a thread." task))

(defmethod execute :before ((task task))
  (assert *current-task* () "No current task active (*CURRENT-TASK* is NIL)."))

(defmethod execute :around ((new-task abstract-task))
  ;; Make sure not to get suspended while setting up the new task;
  ;; otherwise SETUP could push NEW-TASK onto CHILD-TASKS of
  ;; PARENT-TASK before NEW-TASK is actually running. As NEW-TASK
  ;; would not be running, it could of course not act on the messages
  ;; propagated from PARENT-TASK---hence PARENT-TASK would hang in
  ;; case the suspension event is synchronized.  (Spawning a new
  ;; thread involves synchronization where a deadline could get
  ;; signaled.)
  (without-scheduling (call-next-method)))

(defmethod execute ((new-task abstract-task))
  (let ((parent-task *current-task*))
    (setup new-task parent-task)
    (with-slots (name thread thread-fun parent-barrier constraints status) new-task
      (flet ((task-initial-function ()
               (setf thread (current-thread))
               (let ((*current-task* new-task)
                     (*current-path* (task-path new-task)))
                 (unwind-protect-case (abortedp)
                     (event-loop new-task
                                 (make-event `(:execute ,thread-fun
                                                        ,constraints)))
                   (:always
                    (when abortedp
                      (%teardown new-task :failed
                                 "Aborted." "Parent aborted."
                                 t))
                    (log-event
                      (:context "- ~:[FINISH~;ABORTED~] -" abortedp)
                      (:display "~S, ~_~S"
                                (value (status new-task))
                                (result new-task))
                      (:tags :finish)))))))
        (spawn-thread name (tv-closure new-task parent-task
                                       #'task-initial-function)))
      (wait-for (fl-not (fl-eql status :created))))))

(defun tv-closure (task parent-task continuation)
  "Establish bindings for task variables and initialize them."
  (multiple-value-bind (unbound-vars bound-vars bound-vals)
      (loop for (sym . fun) in *task-local-variables*
            if fun
              collect sym into bound and
              collect (funcall fun task parent-task (symbol-value sym))
                into vals
            else
              collect sym into unbound
            finally
              (return (values unbound bound vals)))
    (named-lambda task-variable-closure ()
      (progv bound-vars bound-vals
        (progv unbound-vars nil
          (funcall continuation))))))

;;; These are task-local so the test suite can use them to get all the
;;; tasks spawned during the extent of a test run.

(define-task-variable *save-tasks* nil
  "When t, every task to be executed is pushed to *tasks*.")

(define-task-variable *tasks* (make-queue)
  "Queue of all running tasks."
  :type queue)

(defun list-saved-tasks ()
  (list-queue-contents *tasks*))

(defun clear-saved-tasks ()
  (setf *tasks* (make-queue)))

(defun kill-saved-tasks ()
  (loop for task in (list-queue-contents *tasks*)
        do (kill-thread (slot-value task 'thread)))
  (clear-saved-tasks))

(defmethod execute :after ((new-task abstract-task))
  (when *save-tasks*
    (enqueue new-task *tasks*)))

;;;; Events

;;; EVENTs are used to induce status transitions of TASKs, mostly
;;; among each other, but also by themselves alone.

;;; Usually EVENTs are used for communication between a parent task
;;; and its children, to inform the children how to act due to some
;;; behaviour in the parent task.

;;; EVENTs have the optional property of being ``synchronized'',
;;; indicated by the SYNC slot. Synchronized events mean that the
;;; sender of the event is supposed to wait until the event has
;;; propagated to the leafs of the tree the recipient is root of.

(defstruct (event (:constructor %make-event (tag values reason sync)))
  (tag    (required-argument 'tag) :type keyword)
  (reason nil                      :type (or null simple-string))
  (values nil                      :type list)
  (sync   nil                      :type boolean))

(deftype event-designator ()
  "An event-designator is a either a keyword naming the event, or a
   list whose car is such a keyword and whose cdr is a list of
   values. The list of values represent additional parameters of the
   event and can be destructured in DESTRUCTURE-EVENT."
  `(or keyword (cons keyword *)))

(declaim (inline make-event))
(defun make-event (event-designator &optional reason sync)
  (let ((designator (ensure-list event-designator)))
    (%make-event (car designator) (cdr designator) reason sync)))

;;; Use a more concise print representation than the default for sake
;;; of the logging facility.
(defmethod print-object ((event event) stream)
  (with-slots (tag reason values sync) event
    (print-unreadable-object (event stream :type t :identity nil)
      (format stream "~S~@[ ~:_~S~]~:[~; (:SYNC)~]"
              (if values
                  (cons tag values)
                  tag)
              reason
              sync))))

(defmacro destructure-event ((event &key reason sync) &body clauses)
  "Example:

      (let ((event (make-event `(:foo 1 2 3) \"Odd Reason\" t)))
        (destructure-event (event :reason R :sync S)
          ((:foo X Y Z)
           (values X Y Z R S))))

               ==> 1, 2, 3, \"Odd Reason\", T
  "
  (let ((n-event (gensym "EVENT+")))
    (multiple-value-bind (seen-tags case-clauses)
        (loop for ((tag . arglist) . body) in clauses
              collect tag into seen
              collect `(,tag (destructuring-bind ,arglist
                                 (event-values ,n-event)
                               ,@body)) into clauses
              finally
              (return (values seen clauses)))
      `(let* ((,n-event ,event)
              ,@(when reason `((,reason (event-reason ,n-event))))
              ,@(when sync   `((,sync   (event-sync ,n-event)))))
         (declare (type event ,n-event))
         (case (event-tag ,n-event)
           ,@case-clauses
           (t
            (error "DESTRUCTURE-EVENT fall through on ~S.~%~
                    Expected one of: ~@<~{~S~^, ~}~:>"
                   ,n-event ',seen-tags)))))))

;;; MESSAGEs are envelopes for EVENTs; we do not send EVENTs directly
;;; so we have a container to collect additional meta data.

;;; At the moment, we store information about the sender, and
;;; recipient task for logging purposes.

;;; TODO: We could also store a timestamp when the message was
;;; created. This may be useful to figure out the time difference
;;; between sending a message, and when it's received/processed.  This
;;; again may be useful to figure out good values for the
;;; +TIME-QUANTUM+ below.

(defstruct message
  (from    (required-argument 'from)    :type abstract-task)
  (to      (required-argument 'to)      :type abstract-task)
  (content (required-argument 'content) :type event))

(defun send-event (task event &optional no-sync)
  "Send `event' to `task'.
   If `no-sync' is true, no synchronization is performed on `event',
   the decision to do so is left to the caller."
  (send-message (task-message-queue task)
                (make-message :from (current-task)
                              :to task
                              :content event))
  (log-event
    (:context "SENT -> ~A" (task-abbreviated-name task))
    (:display "~S" event)
    (:tags :send-event))
  (unless no-sync
    (synchronize-on-event event (list task))))

(defun receive-event (&key wait)
  "If an event is currently pending, return it, otherwise return NIL.
   If `wait' is true, block until an event arises."
  (let* ((mailbox (task-message-queue *current-task*)))
    (multiple-value-bind (message ok?)
        (if wait
            (values (receive-message mailbox) t)
            (receive-message-no-hang mailbox))
      (when ok?
        (let ((from  (message-from message))
              (event (message-content message)))
            (log-event
              (:context "RECV <- ~A" (task-abbreviated-name from))
              (:display "~S" event)
              (:tags :receive-event))
            event)))))

(defun propagate-event (task event)
  "Propagate `event' down to the childs of `task'."
  (log-event
    (:context "PROPAGATE-EVENT")
    (:display "~S" event)
    (:tags :propagate-event))
  (let ((child-tasks (child-tasks task)))
    (dolist (child child-tasks)
      (send-event child event t))
    (synchronize-on-event event child-tasks)))

(defun synchronize-on-event (event child-tasks)
  "If `event' is a synchronized event, wait until `child-tasks'
   acted on it."
  (flet ((synced (event)
           (let ((states (list (ecase (event-tag event)
                                 (:suspend   :suspended)
                                 (:wakeup    :running)
                                 (:fail      :failed)
                                 (:evaporate :evaporated))
                               ;; Tasks may succeed, fail or get
                               ;; evaporated at any time; in
                               ;; particular, before receiving an
                               ;; event.
                               :succeeded
                               :evaporated
                               :failed)))
             #'(lambda (task)
                 (fl-member (status task) states)))))
    (when (event-sync event)
      (wait-for (apply #'fl-and (mapcar (synced event) child-tasks))))))


;;;; Scheduler

;;; The scheduler is responsible to let us periodically enter the
;;; event loop.

(defparameter +time-quantum+ 0.005
  "Period in seconds at which an entrance into the event loop is
   scheduled.")

(define-condition time-quantum-exhausted () ()
  (:documentation
   "Always signaled with an associated restart
    CONTINUE-WITH-ADJUSTED-TIME-QUANTUM."))


(declaim (inline timeout-to-internal-time))
(defun seconds-to-internal-time (timeout)
  (sb-impl::seconds-to-internal-time timeout))

(defun adjust-time-quantum (&optional (new +time-quantum+))
  "Renew the current time quantum."
  ;; We can't assert SCHEDULING-ENABLED-P here because we need to use
  ;; this function within the event loop where scheduling is
  ;; disabled. Still the following check seems rather sensible: 
  (assert (thread-local-binding-p 'sb-impl::*deadline*))
  (setf sb-impl::*deadline* (+ (get-internal-real-time)
                               (seconds-to-internal-time new)))
  (setf sb-impl::*deadline-seconds* new))

(defun continue-with-adjusted-time-quantum (&optional new-quantum c)
  (let ((restart (find-restart 'continue-with-adjusted-time-quantum c)))
    (assert restart)
    (invoke-restart restart (or new-quantum +time-quantum+))))


(defvar *scheduling-enabled* nil)

(defun call-with-scheduling (thunk)
  "Execute `thunk', and signal TIME-QUANTUM-EXHAUSTED if the current
   time quantum is exhausted."
  ;; Disallow nesting. We don't need it, and allowing it would require
  ;; some thinking regarding the additional *DEADLINE* binding that
  ;; would be established by the nesting, and its interplay with
  ;; ADJUST-TIME-QUANTUM.
  (assert (not *scheduling-enabled*))
  (handler-bind ((sb-sys:deadline-timeout
                  #'(lambda (c)
                      (unless *scheduling-enabled*
                        (sb-sys:defer-deadline +time-quantum+ c))
                      (restart-bind ((continue-with-adjusted-time-quantum
                                      #'sb-sys:defer-deadline))
                        (let ((*scheduling-enabled* nil))
                          (assert-no-returning
                            ;; We don't return because handlers are
                            ;; supposed to invoke the restart above.
                            (signal 'time-quantum-exhausted)))))))
    (sb-sys:with-deadline (:seconds +time-quantum+)
      (let ((*scheduling-enabled* t))
        (funcall thunk)))))

(defun call-without-scheduling (thunk)
  ;; We do not bind SB-IMPL::*DEADLINE* to NIL because it would
  ;; prevent using ADJUST-TIME-QUANTUM during the execution of
  ;; THUNK. (We do not currently need this, but I guess it's better to
  ;; do it right from the beginning.)
  (prog1
      (let ((*scheduling-enabled* nil))
        (funcall thunk))
    (check-time-quantum)))

(defun check-time-quantum ()
  "Check whether we already exceeded our current time quantum."
  (when *scheduling-enabled*
    (sb-sys:decode-timeout nil)))

(defun signal-time-quantum-exhausted ()
  (when *scheduling-enabled*
    ;; We cannot signal a TIME-QUANTUM-EXHAUSTED directly here
    (sb-sys:signal-deadline)))

(defun scheduling-enabled-p ()
  (let ((enabledp *scheduling-enabled*))
    (prog1 enabledp
      ;; Just some sanity checks.
      ;; (if enabledp
      ;;     (assert sb-impl::*deadline*)
      ;;     (assert (null sb-impl::*deadline*)))
      )))

(defun scheduling-disabled-p ()
  (not (scheduling-enabled-p)))


;;;; Event Loop

(defvar *suspension-handlers* nil)

(defun call-on-suspension (on-suspension-thunk body-thunk)
  (let ((*suspension-handlers* (cons on-suspension-thunk
                                     *suspension-handlers*)))
    (funcall body-thunk)))

(deftype continuation ()
  `(function () *))

(defvar *suspension-unwind* nil)
(declaim (type (or null (function (continuation) *)) *suspension-unwind*))

(defun %unwind-on-suspension (body-thunk after-suspension-continuation)
  "If a suspension occurs during the execution of `body-thunk',
   unwind out of `body-thunk', continue the suspension process, and
   after wakeup execute `after-suspension-continuation'."
  (prog ((after-unwind-continuation))
     ;; Even in the unlikely case of nesting, we only have to remember
     ;; one here, namely the outermost place.
     (let ((*suspension-unwind* (or *suspension-unwind*
                                    #'(lambda (c)
                                        (setq after-unwind-continuation c)
                                        (go :unwind)))))
       (return (funcall body-thunk)))
   :unwind
     (funcall after-unwind-continuation)        ; continue suspension process
     (funcall after-suspension-continuation)))  ; continue after wakeup


(defun unwind-and-continue (handlers unwind continuation)
  "Unwind using `unwind', then invoke `handlers', and call
  `continuation'."
  (declare (type (or null (function (continuation) *)) unwind))
  (declare (type continuation continuation))
  (if unwind
      (funcall unwind
               (named-lambda continue-suspension ()
                   ;; We unwound the LET form binding this to NIL in
                   ;; CALL-WITH-SCHEDULING, so reestablish it.
                   (let ((*scheduling-enabled* nil))
                     ;; Call handlers after unwind as a) it shouldn't
                     ;; make a difference (an on-suspension handler
                     ;; can't know where a suspension will happen, so it
                     ;; shouldn't rely on the exact dynamic environment
                     ;; at the point of suspension), b) it's safer here
                     ;; because we now went out of critical sections.
                     (mapc #'funcall handlers)
                     ;; After unwind, the old continue restart will be
                     ;; gone, so we have to reestablish a new one. (This
                     ;; is where EVENT-LOOP will get us to in case of
                     ;; :WAKEUP.)
                     (restart-case (funcall continuation)
                       (continue-with-adjusted-time-quantum (seconds)
                         ;; What the old restart did for us, we now have
                         ;; to do ourselves:
                         (adjust-time-quantum seconds)
                         ;; We return to wherever UNWIND has brought us to.
                         )))))
      (progn
        (mapc #'funcall handlers)
        ;; The old continue restart is still active.
        (funcall continuation))))

(defun unwind-and-teardown (status result reason sync)
  (assert-no-returning
    (invoke-restart 'teardown status result reason sync)))


(defun call-with-event-loop (thunk constraints)
  "Execute `thunk', and periodically enter the event-loop."
  (let ((current-task *current-task*))
    (handler-bind ((time-quantum-exhausted
                    #'(lambda (c)
                        (declare (ignore c))
                        (event-loop current-task))))
      (call-with-scheduling
       #'(lambda ()
           (handler-bind ((plan-failure
                           #'(lambda (c)
                               (invoke-event-loop (make-event `(:fail ,c)))))
                          (error
                           #'(lambda (c)
                               (restart-case
                                   (if *debug-on-lisp-errors*
                                       (invoke-debugger c)
                                       (invoke-restart 'rethrow))
                                 (rethrow ()
                                   :report (lambda (stream)
                                             (format stream "Rethrow condition `~s'" c))
                                   :interactive (lambda () nil)
                                   (invoke-event-loop
                                    (make-event
                                     `(:fail ,(make-condition
                                               'common-lisp-error-envelope
                                               :error c)))))))))
             (when constraints
               ;; We resolve constraints here when scheduling is
               ;; enabled, so we'll be able to act on messages.
               (wait-for (apply #'fl-and constraints)))
             (multiple-value-prog1 (funcall thunk)
               ;; Before returning success, enter event loop for a
               ;; last time and check for pending messages.
               (invoke-event-loop))))))))

(defun invoke-event-loop (&optional event)
  (assert (scheduling-enabled-p))
  (when event (send-event *current-task* event))
  (signal-time-quantum-exhausted))

(defun event-loop (current-task &optional event wait-for-event)
  (assert (scheduling-disabled-p))
  (let ((event (or event
                   (receive-event :wait wait-for-event)
                   (load-time-value (make-event :nop) t))))
    (log-event
      (:context "EVENT-LOOP")
      (:display "~S" event)
      (:tags :event-loop))
    (destructure-event (event :reason reason :sync sync)
      ((:nop)
       (assert-status current-task :running)
       (continue-with-adjusted-time-quantum))
      ((:execute thunk constraints)
       (assert-status current-task :created)
       (change-status current-task :running)       
       (restart-case
           (%teardown current-task
                      :succeeded
                      (multiple-value-list
                       (call-with-event-loop thunk constraints))
                      "Parent succeeded."
                      t)
         (teardown (status result reason sync)
           :report (lambda (stream)
                     (format stream "Tear down ~S" current-task))
           :interactive (lambda ()
                          (let ((reason "Manual teardown."))
                            (list :evaporated nil reason nil)))
           (%teardown current-task status result reason sync))))
      ((:suspend)
       (unwind-and-continue *suspension-handlers*
                            *suspension-unwind*
                            ;; This here is the "current" continuation,
                            ;; or rather: what's supposed to happen
                            ;; right after unwinding.
                            #'(lambda ()
                                (propagate-event current-task event)
                                (change-status current-task :suspended)
                                (assert-no-returning
                                  ;; We won't ever return because
                                  ;; wakeup will perform an nlx via
                                  ;; the continue restart.
                                  (event-loop current-task nil t)))))
      ((:wakeup)
       (propagate-event current-task event)
       (change-status current-task :running)
       (continue-with-adjusted-time-quantum))
      ((:evaporate)
       (unwind-and-teardown :evaporated nil reason sync))
      ((:fail condition)
       ;; NB: As we handle failures via messages, we automatically
       ;; process pending messages before performing the failure.
       (unwind-and-teardown :failed condition reason sync)))))

(defun %teardown (task final-status result reason sync)
  (log-event
    (:context "TEARDOWN")
    (:display "~S, ~:_~S, ~:_~S, ~:_~S"
              final-status result reason sync)
    (:tags :teardown))
  (propagate-event task (make-event :evaporate reason sync))
  (setf (slot-value task 'result) result)
  (change-status task final-status))


;;;; Misc

(defun log-gc-event ()
  (let ((sb-impl::*deadline* nil)
        (sb-impl::*deadline-seconds* nil))    
    (log-event
      (:context "GC")
      (:display "new dynamic usage: ~10:D bytes" (sb-kernel:dynamic-usage))
      (:tags :gc))))

;;; FIXME: enabling it resulted regularly in whole-image deadlocks on
;;; SBCL 1.0.38, Linux x86-32 when running the test suite with
;;; +LOG-VERY-VERBOSE+. So let's better not enable it by default.
#+nil
(push 'log-gc-event sb-ext:*after-gc-hooks*)
