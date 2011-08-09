;;;
;;; Copyright (c) 2010, Tobias C Rittweiler <trittweiler@common-lisp.net>
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


(in-package :cpl-tests)

;;;; Customizations

(defparameter +timeout+ 5.0
  "Default timeout per run.")

(defparameter +n-runs+ 10
  "Default number of runs a test should be repeated.")

(defvar *compile-tests-at* :definition-time
  "We want the tests to be compiled at definition time so SBCL will
   emit warnings at compilation time to catch typos, etc. However,
   compiling them at definition time, means that tests have to be
   recompiled in case macros are changed.")
(declaim (type (member :definition-time :run-time) *compile-tests-at*))

(define-task-variable *failure-behaviour* nil
  "Alist of type-specifier (for a condition) and one of the symbols
   NIL, :BREAK, :TEARDOWN. See documentation string of
   DEFINE-CRAM-TEST for explanation."
  :type list)

(defvar *ignore-skips* nil
  "If true, execute tests even though they specified (:SKIP).")

(defvar *include-backtrace* nil
  "If true, include backtraces for failed tasks in test failures.")

(defparameter +backtrace-length+ 20
  "Length of the backtraces.")

;;;; DEFINE-CRAM-TEST

(defmacro define-cram-test (name docstring clauses &body body)
  "Some sugar on top of 5AM:TEST.

During the execution of BODY

  - Errors are caught and turned into test failures.

  - Hangs are caught by an implicitly, or explicitly specified
    timeout.

  - Tasks spawned during the execution of the test are collected. In
    case of an error, or timeout, all these threads collected will be
    killed.

  - The behaviour of plan failures during the test is highly
    customizable via *FAILURE-BEHAVIOUR*:

       It's an alist mapping type-specifier for a condition to a
       symbol representing the desired behaviour: one of
       NIL, :BREAK, or :TEARDOWN.

       For example:

           ((MY-PLAN-FAILURE . :BREAK)
            (OTHER-PLAN-FAILURE . NIL)
            (T . :TEARDOWN))

       The type specifier designates the conditions for which the
       specified behaviour is supposed to hold true.

         NIL        represents the default behaviour (i.e. to propagate
                    plan failures to the task's parent.)

         :BREAK     means to enter the debugger. Notice that the timeout
                    will still be active at that time, so you probably
                    want to recompile the test with a (:TIMEOUT NIL)
                    clause. See below.

         :TEARDOWN  means to tear down the task, which includes
                    evaporation of all childs.

       In the last two cases, :BREAK and :TEARDOWN, a message will be
       logged with tag :DEBUGGER.

  - Furthermore, backtraces are saved, and will be printed in case
    *INCLUDE-BACKTRACE* is T.

  - In case of an error, or timeout, some statistics is printed along
    the error message on how many threads were created, how many
    threads terminated by themselves, how many threads aborted due to
    an error by themselves, and how many threads had to be killed.


CLAUSES can be one of

  (:N-RUNS <n>)

    specifies how many times BODY is executed. Defaults to
    5AM::*NUM-TRIALS* which again defaults to 100.

  (:TIMEOUT <timeout>)

    specifies a timeout in seconds, _per run_. If BODY does not
    terminate until the timeout expired, a test failure is generated.
    Defaults to +TIMEOUT+.

    The timeout may be NIL which means no timeout at all. Useful in
    combination with a *FAILURE-BEHAVIOUR* of :BREAK

  (:GENERATORS <bindings>+)

    specifies bindings for 5AM's random testing facility as per
    5AM:FOR-ALL.

  (:SKIP <reason>)

    specifies to skip over the test. (:SKIP) clauses are ignored in
    case *IGNORE-SKIPS* is true.

A documentation string must be given. Please document what your test
cases are supposed to test."
  (check-type docstring string)
  (labels ((default (x default-value)
             (if (eq x :default) default-value x))
           (assert-default (tag var)
             (assert (eq var :default) () "~S specified twice." tag))
           (parse-clauses (clauses)
             (let ((timeout     :default)
                   (generators  :default)
                   (n-runs      :default)
                   (skip-reason :default))
               (dolist (clause clauses (values
                                        (default timeout     +timeout+)
                                        (default generators  nil)
                                        (default n-runs      +n-runs+)
                                        (default skip-reason nil)))
                 (destructure-case clause
                   ((:timeout x)
                    (assert-default :timeout timeout)
                    (setq timeout x))
                   ((:generators &rest gs)
                    (assert-default :generators generators)
                    (setq generators gs))
                   ((:n-runs n)
                    (assert-default :n-runs n-runs)
                    (setq n-runs n))
                   ((:skip &rest reason)
                    (assert-default :skip skip-reason)
                    (setq skip-reason reason)))))))
    (multiple-value-bind (timeout generators n-runs skip-reason)
        (parse-clauses clauses)
      ;; We want the tests to be compiled at definition time so we
      ;; will get warnings etc. at compile-time; of course, that won't
      ;; take macro
      `(5am:test (,name :compile-at ,*compile-tests-at*)
         (format 5am:*test-dribble* "~&Running ~S: " ',name)
         ,(cond
           ((and skip-reason (not *ignore-skips*))
            `(5am:skip ,@skip-reason))
           (t
            `(let ((5am::*num-trials* ,n-runs))
               (5am:for-all ,generators
                 (run-cram-test ',name ,timeout #'(lambda () ,@body))))))))))


;;;; The glory details

(defun list-spawned-threads (tasks)
  (declare (type queue tasks))
  (loop for task in (list-queue-contents tasks)
        for thread = (slot-value task 'thread)
        do (assert thread ()
                   "~@<Assertion failure in test suite infrastructure. ~
                       Task ~S was on ~S which did not have a thread ~
                       associated with it. Did you mess with CPL ~
                       internals?~@:>"
                   task '*tasks*)
        collect thread))

(defun assert-threads-dead (threads)
  ;; It may take quite _some_ time until a task that is basically dead
  ;; finishes down so that its thread can update its alive-p slot; so
  ;; let's give it some time to do so:
  (dolist (thread threads)
    (join-thread thread :timeout 1.0))
  (let ((alive (remove-if-not #'thread-alive-p threads)))
    (assert (null alive) ()
            "~@<Test returned success, yet the following threads ~
                were still alive: ~2I~:_~{~A~^, ~}.~@:>"
            (mapcar #'thread-name alive))))

(defun shutdown-spawned-tasks (tasks n-repeats)
  (declare (type queue tasks) (type unsigned-byte n-repeats))
  (let ((already-dead    (cons 0 nil))
        (already-aborted (cons 0 nil))
        (killed          (cons 0 nil))
        (unkillable      (cons 0 nil)))
    (flet* ((add (thread box)
             (incf (car box))
             (push (thread-name thread) (cdr box)))
            (shutdown ()
              (loop for task   = (dequeue tasks)
                    for thread = (and task (slot-value task 'thread))
                    while thread do
                    (cond ((not (thread-alive-p thread))
                           (if (eq (join-thread thread) :error)
                               (add thread already-aborted)
                               (add thread already-dead)))
                          ((kill-thread thread)
                           (add thread killed))
                          (t
                           (case (join-thread thread :timeout 0.1)
                             ((:error)   (add thread already-aborted))
                             ((:timeout) (add thread unkillable))
                             (t          (add thread already-dead))))))))
      ;; There's a race at play here: killing a task's thread may have
      ;; some small delay, and during that delay, it may spawn new
      ;; tasks and hence threads. That may happen exactly after
      ;; DEQUEUE returned NIL. We kill N-REPEATS times so the
      ;; likelyhood of the race is kept very very small.
      (loop repeat n-repeats do (shutdown) (sleep 0.05))
      (values (reduce #'+ (list already-dead already-aborted
                                killed unkillable)
                      :key #'car)
              killed unkillable
              already-dead already-aborted))))

(defun serious-test-failure (condition backtraces tasks)
  (multiple-value-bind (total killed unkillable already-dead already-aborted)
      (shutdown-spawned-tasks tasks 3)
    (symbol-macrolet
        ((n-killed                (car killed))
         (n-unkillable            (car unkillable))
         (n-already-dead          (car already-dead))
         (n-already-aborted       (car already-aborted))
         (killed-threads          (cdr killed))
         (unkillable-threads      (cdr unkillable))
         (already-dead-threads    (cdr already-dead))
         (already-aborted-threads (cdr already-aborted)))
      ;; The per-line-prefix spaces are chosen so they fit well with how
      ;; 5am reports failures.
      (5am:fail
       "~%~
        ~@<    ~@;~
          Preempted due to ~S:~@:_~
              ~<   ~@;\"~A\"~:>~@:_~@:_~
          Backtraces: ~:[(*INCLUDE-BACKTRACE* is NIL)~;~]~@:_~
              ~<   ~@;~{-- ~A --~@:_~:[ ...~;~:*~A~]~^~@:_~}~:>~@:_~@:_~
          Summary:~1I~@:_~
            ~3D tasks total~
            ~:[~@:_~3D terminated before preemption: ~<~@{~A~^, ~:_~}~:>~;~2*~]~
            ~:[~@:_~3D aborted before preemption:    ~<~@{~A~^, ~:_~}~:>~;~2*~]~
            ~:[~@:_~3D killed after preemption:      ~<~@{~A~^, ~:_~}~:>~;~2*~]~
            ~:[~@:_~3D unkillable ghosts:            ~<~@{~A~^, ~:_~}~:>~;~2*~]~
        ~:>"
       condition (list condition)
       *include-backtrace*
       (list (hash-table-plist backtraces))
       total
       (zerop n-already-dead)    n-already-dead     already-dead-threads
       (zerop n-already-aborted) n-already-aborted  already-aborted-threads
       (zerop n-killed)          n-killed           killed-threads
       (zerop n-unkillable)      n-unkillable       unkillable-threads))))

;;; FIXME: Maybe this should be put into cram-utilities, in a file
;;; impldep.lisp.
(defun backtrace ()
  (with-output-to-string (s)
    (sb-debug:backtrace +backtrace-length+ s)))

(define-task-variable *debugger-hook* *debugger-hook*
    "Make *DEBUGGER-HOOK* a task-local variable shared between parent
     and child task, so we can customize test-local behaviour in
     RUN-CRAM-TEST."
  :ignore-package-locks t)

(defun run-cram-test (name timeout thunk)
  (let ((*save-tasks* t)
        (*tasks* (make-queue :name (format nil "*TASKS* (TEST ~A)" name)))
        (*break-on-plan-failures* '(satisfies test-failure-behaviour))
        (backtraces (make-synchronized-hash-table :test #'equal)))
    ;; N.b. unwinding by HANDLER-CASE out of the WITHOUT-TIMEOUT is
    ;; important so we won't get caught by a timeout during the
    ;; execution of SERIOUS-TEST-FAILURE.
    (handler-case
        (handler-bind ((serious-condition
                        #'(lambda (c)
                            (declare (ignore c))
                            (store-backtrace backtraces (current-task)))))
          (multiple-value-prog1
              (let ((*debugger-hook* (make-cram-debugger backtraces)))
                ;; We try to enforce the timeout with a deadline
                ;; first because unwinding from such a place is
                ;; deemed to be safer.
                (if timeout
                    (sb-ext:with-timeout (+ timeout 0.25)
                      (sb-sys:with-deadline (:seconds timeout)
                        (funcall thunk)))
                    (funcall thunk)))
            ;; There's a small race here, too: if the test nominally
            ;; passed but there's a task left over which spawns new
            ;; tasks right after the LIST-SPAWNED-THREADS here. However,
            ;; I don't think this race can ever fully be overcome (just
            ;; as in SHUTDOWN-SPAWNED-TASKS.) Anyhow, all this is
            ;; supposed to be "just" sanity checks anyway.
            (assert-threads-dead (list-spawned-threads *tasks*))
            ;; Tasks tear down with status :EVAPORATED right after
            ;; entering the debugger. It could be that the test case
            ;; somehow passes nontheless, so make sure to provoke a
            ;; test failure if BACKTRACES is not empty.
            (assert-no-entries backtraces)))
      (serious-condition (c)
        (serious-test-failure c backtraces *tasks*)
        c))))


(defun assert-no-entries (backtraces)
  (assert (zerop (hash-table-count backtraces)) ()
          "The following tasks entered the debugger: ~{~A~^, ~}"
          (mapcar #'car (hash-table-alist backtraces))))

(defun store-backtrace (backtraces task)
  (setf (gethash (name task) backtraces)
        (and *include-backtrace* (backtrace))))

(defun test-failure-behaviour (condition)
  ;; Regardless of what the specific failure-behaviour is, we must
  ;; return true here so that we'll enter the debugger.  The
  ;; debugger-hook will then act on the specific failure behaviour
  ;; appropriately.
  (and (get-failure-behaviour condition) t))

(defun get-failure-behaviour (condition)
  (cdr (assoc condition *failure-behaviour* :test #'typep)))

(defun make-cram-debugger (backtraces)
  (let ((real-debugger *debugger-hook*))
    (named-lambda cram-debugging-hook
        (condition self)
      (declare (ignore self))
      (assert *current-task*)           ;  we can only affect tasks.
      (log-debug-event condition)
      (store-backtrace backtraces *current-task*)
      (case (get-failure-behaviour condition)
        ((:break)
         (funcall real-debugger condition real-debugger))
        (t
         (cpl-impl::unwind-and-teardown :failed
                                        condition
                                        "Parent entered debugger."
                                        nil))))))

(defun log-debug-event (condition)
  (cpl-impl::log-event
    (:context "*** ENTERED DEBUGGER ***")
    (:display "~S: ~_\"~A\"" condition (princ-to-string condition))
    (:tags :debugger)))

