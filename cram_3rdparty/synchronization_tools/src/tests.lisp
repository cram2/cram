
(in-package :tcr.sync)

;;;; Utils

(defparameter +n-threads+ 250)
(defparameter +timeout+ 120.0)

(defun kill-thread (thread)
  (when (thread-alive-p thread)
    (terminate-thread thread)))

(defmacro with-thread-diagnostics (() &body body)
  `(handler-bind ((serious-condition
                   #'(lambda (c)
                       ;; Make sure to write at once.
                       (write-string
                        (format nil 
                                "~&Thread ~S entering debugger... ~%  ~A~%"
                                *current-thread* c)))))
     ,@body))

(defmacro with-test-setup ((&key timeout) &body body)
  `(let ((%spawned-threads% (sb-queue:make-queue)))
     (macrolet ((spawned-threads ()
                  `(sb-queue:list-queue-contents %spawned-threads%))
                (spawn ((&rest initial-bindings) &body body)
                  `(sb-queue:enqueue
                    (make-thread (let ,,'initial-bindings
                                   #'(lambda ()
                                       (with-thread-diagnostics ()
                                         ,@,'body))))
                    %spawned-threads%))
                (named-spawn (name (&rest initial-bindings) &body body)
                  `(sb-queue:enqueue
                    (make-thread (let ,,'initial-bindings
                                   #'(lambda ()
                                       (with-thread-diagnostics ()
                                         (block ,name ,@,'body))))
                                 :name (string ',name))
                    %spawned-threads%)))
       (handler-case
           (sb-ext:with-timeout ,(or timeout '+timeout+) ,@body)
         (serious-condition (c)
           (mapc #'kill-thread (spawned-threads))
           c)))))

(defmacro define-synchronization-test (name form &rest values)
  "Like DEFTEST but FORM is wrapped in a WITH-TIMEOUT."
  `(deftest ,name (with-test-setup (:timeout +timeout+) ,form)
     ,@values))

(defmacro spawn ((&rest initial-bindings) &body body)
  (declare (ignore initial-bindings body))
  (error "SPAWN must be used within DEFINE-SYNCHRONIZATION-TEST."))

(defmacro named-spawn (name (&rest initial-bindings) &body body)
  (declare (ignore name initial-bindings body))
  (error "NAMED-SPAWN must be used within DEFINE-SYNCHRONIZATION-TEST."))

(defmacro signals-condition-p (name &body body)
  `(handler-case (prog1 nil ,@body)
     (condition (c) (typep c ,name))))

(defun iota (n)
  (loop for i from 0 below n collect i))

(defun random-elt (list)
  (nth (random (length list)) list))

(defun all (predicate list &key (key #'identity))
  (every #'(lambda (x) (funcall predicate (funcall key x))) list))

(defun none (predicate list &key (key #'identity))
  (notany #'(lambda (x) (funcall predicate (funcall key x))) list))

(defun boolean-true (x) (eq x t))

(defun finished-p (barrier)
  (declare (barrier barrier))
  (eq (barrier-state barrier) :finished))

(defun broken-p (barrier)
  (declare (barrier barrier))
  (eq (barrier-state barrier) :broken))

(defun ignore-deadline (c)
  (sb-sys:defer-deadline +timeout+ c))

;;;; Synchronized Collectectors

(deftest with-buckets.1
    (with-buckets collect-into (even odd)
      (loop for i from 0 below 10
            if (oddp i) do (collect-into odd i)
            else        do (collect-into even i))
      (values even odd))
  (0 2 4 6 8)
  (1 3 5 7 9))

(deftest with-buckets.2
    (with-buckets collect-into ((even :synchronized t)
                                 (odd :synchronized t))
      (loop for i from 0 below 10
            if (oddp i) do (collect-into odd i)
            else        do (collect-into even i))
      (values even odd))
  (0 2 4 6 8)
  (1 3 5 7 9))

(deftest with-buckets.3
    (with-buckets collect-into (quotients remainders)
      (loop for i from 0 below 10 do
            (collect-into (values quotients remainders)
              (floor i 2)))
      (values quotients remainders))
  (0 0 1 1 2 2 3 3 4 4)
  (0 1 0 1 0 1 0 1 0 1))

(define-synchronization-test with-buckets.4
    (let ((n +n-threads+))
      (with-buckets collect-into ((bucket :synchronized t))
        (dotimes (i n)
          (spawn ((i i))
            (sleep (random 1.0))
            (collect-into bucket i)))
        (mapc #'join-thread (spawned-threads))
        (equal (sort bucket #'<) (iota n))))
  t)


;;;; Synchronization Barriers

;;; Create couple of threads that put a counter into a bucket; use
;;; barriers as synchronization points among the helper threads
;;; themselves, and also between the helper threads and the test
;;; driving thread.
(define-synchronization-test with-synchronization-barrier.1
  (let ((n +n-threads+))
    (with-buckets collect-into ((bucket :synchronized t))
      (with-synchronization-barriers ((before-collect (1+ n))
                                      (after-collect  (1+ n)))
        (dotimes (i n)
          (spawn ((i i))
            (sleep (random 1.0))
            (enter-barrier before-collect)
            (collect-into bucket i)
            (enter-barrier after-collect)))
        (values bucket
                (finished-p (enter-barrier before-collect))
                (finished-p (enter-barrier after-collect))
                (equal (sort bucket #'<) (iota n))))))
  nil
  t
  t
  t)

;;; Make couple of threads synchronize on a barrier, then break that
;;; barrier. (Notice how some threads may try to enter the barrier
;;; after it was broken, too.) Check that all these threads received a
;;; BROKEN-BARRIER error, and none crossed the barrier successfully.
(define-synchronization-test with-synchronization-barrier.2
  (let ((n +n-threads+))
    (with-buckets collect-into ((crossed :synchronized t)
                                (broken  :synchronized t))
      (with-synchronization-barriers
          ((ready        (1+ n))
           (to-be-broken (1+ n))
           (finish       (1+ n)))
        (dotimes (i n)
          (spawn ((i i))
            (handler-case (progn
                            (enter-barrier ready)
                            (sleep 0.1)
                            (enter-barrier to-be-broken)
                            (collect-into crossed i)
                            (enter-barrier finish))
              (broken-barrier (c)
                (assert (eq (broken-barrier--barrier c) to-be-broken))
                (collect-into broken i)
                (enter-barrier finish)))))
        (values
         (finished-p (enter-barrier ready))
         (broken-p   (break-barrier to-be-broken))
         (finished-p (enter-barrier finish))
         (equal (sort broken #'<) (iota n))
         crossed))))
  t
  t
  t
  t
  nil)


;;; Very basic test to assure that blocking on a barrier takes
;;; deadlines into account.
(define-synchronization-test synchronization-barriers+deadlines.1
    (with-synchronization-barrier (barrier 2)
      (let ((tester (spawn ()
                      (signals-condition-p 'sb-sys:deadline-timeout
                        (sb-sys:with-deadline (:seconds 0.1)
                          (enter-barrier barrier :if-broken :punt))))))
        (thread-yield)
        (sleep 0.5)
        (break-barrier barrier)
        (join-thread tester)))
  t)

;;; Here we assure that a barrier will still function properly after
;;; it's been temporarily interrupted by a deadline, and the handler
;;; has choosen to continue.
(define-synchronization-test synchronization-barriers+deadlines.2
    (flet ((test (what-to-do)
             (with-synchronization-barriers ((before 2 :name "before")
                                             (main   2 :name "main"))
               (let ((tester
                      (spawn ()
                        (handler-bind ((sb-sys:deadline-timeout
                                        #'(lambda (c)
                                            (enter-barrier before)
                                            (ignore-deadline c))))
                          (sb-sys:with-deadline (:seconds 0.1)
                            (and (enter-barrier main :if-broken :punt) t))))))
                 (enter-barrier before)
                 (funcall what-to-do main)
                 (join-thread tester)))))
      (values
       (test #'enter-barrier)
       (test #'break-barrier)))
  t
  nil)

;;; Here we assure that a) a deadline handler may still be holding on
;;; a mutex when reacquiring the lock was successfull after waking up
;;; in CONDITION-WAIT; also we assure b) that a deadline is likewise
;;; signaled if reacquiring the mutex did _not_ work after waking up
;;; in CONDITION-WAIT -- in that case the deadline handler ends up not
;;; holding the lock.
;;;
;;; These are current semantics of SBCL, I'm not sure they make sense.
(define-synchronization-test synchronization-barriers+deadlines.3
    (with-synchronization-barriers ((A-ready 2)
                                    (B-ready 2)
                                    (main    3))
      (named-spawn A ((hold? nil))
        (handler-bind ((sb-sys:deadline-timeout
                        #'(lambda (c)
                            ;; this will be T because it's just A out there.
                            (setq hold? (holding-mutex-p (barrier-mutex main)))
                            (enter-barrier A-ready)
                            (enter-barrier B-ready)
                            (ignore-deadline c))))
          (sb-sys:with-deadline (:seconds 0.1)
            (enter-barrier main)
            hold?)))
      (enter-barrier A-ready)
      (named-spawn B ((hold? nil))
        (handler-bind ((sb-sys:deadline-timeout
                        #'(lambda (c)
                            ;; this will be NIL because A's holding the lock.
                            (setq hold? (holding-mutex-p (barrier-mutex main)))
                            (enter-barrier B-ready)
                            (ignore-deadline c))))
          (sb-sys:with-deadline (:seconds 0.1)
            (enter-barrier main)
            hold?)))
      (enter-barrier main)
      (values-list (mapcar #'join-thread (spawned-threads))))
  t
  nil)

(define-synchronization-test enter-barrier+unwinding.due-to-deadline.1
    (with-synchronization-barrier (barrier 2)
      (handler-case (sb-sys:with-deadline (:seconds 0.01)
                      (enter-barrier barrier))
        (sb-sys:deadline-timeout ()
          (barrier-count barrier))))
  0)

;;; In ENTER-BARRIER: let's say a thread signals that the threshold is
;;; met; however, before that signal is processed, a waiting thread
;;; may be interrupted resulting in an unwind resulting in a
;;; retrospective decrement of the COUNT.
;;;
;;; We test here that other waiting threads are waked up
;;; nontheless. As a result, a barrier may end up in state :FINISHED
;;; with its COUNT < its THRESHOLD.
;;;
;;; Different behaviour does not make sense because for the signaling
;;; thread, the barrier definitively finished.
(define-synchronization-test enter-barrier+unwinding.due-to-deadline.2
    (with-synchronization-barriers ((all-ready 4)
                                    (barrier 4))
      (named-spawn A ()
        (handler-bind ((sb-sys:deadline-timeout
                        #'(lambda (c)
                            ;; A will be holding the mutex here.
                            (enter-barrier all-ready)
                            ;; After ALL-READY, go back into
                            ;; ENTER-BARRIER to release the mutex.
                            (ignore-deadline c))))
          (sb-sys:with-deadline (:seconds 0.1)
            (enter-barrier barrier))))
      (named-spawn B ()
        (handler-bind ((sb-sys:deadline-timeout
                        #'(lambda (c)
                            (declare (ignore c))
                            ;; B will not hold the mutex; we come here
                            ;; from the UWP cleanup clause in
                            ;; CONDITION-WAIT.
                            (enter-barrier all-ready)
                            ;; wait longer than D but less than C, so
                            ;; C will wake up after we unwound and
                            ;; decreased the BARRIER-COUNT
                            ;; retrospectively..
                            (sleep 0.2)
                            (return-from b))))
          (sb-sys:with-deadline (:seconds 0.2)
            (enter-barrier barrier))))
      (named-spawn C ()
        (handler-bind ((sb-sys:deadline-timeout
                        #'(lambda (c)
                            ;; Like in B.
                            (enter-barrier all-ready)
                            ;; Wait longer than B, so when we go back
                            ;; into ENTER-BARRIER, we'll return from
                            ;; CONDITION-WAIT (due to wake up by D)
                            ;; and reevaluate the looping condition.
                            (sleep 0.3)
                            (ignore-deadline c))))
          (sb-sys:with-deadline (:seconds 0.3)
            (enter-barrier barrier))))
      (named-spawn D ()
        (enter-barrier all-ready)
        ;; After ALL-READY, sleep a bit to let A release the mutex
        ;; so we can enter the barrier and fill its threshold.
        (sleep 0.1)
        (enter-barrier barrier))
      (mapc #'join-thread (spawned-threads))
      (values (barrier-state barrier) (barrier-count barrier)))
  :finished
  3)

(define-synchronization-test enter-barrier+unwinding.due-to-interrupt.1
    (with-synchronization-barrier (barrier 2)
      (handler-case (sb-ext:with-timeout 0.01
                      (enter-barrier barrier))
        (sb-ext:timeout ()
          (barrier-count barrier))))
  0)

(define-synchronization-test enter-barrier+unwinding.due-to-interrupt.2
    (with-synchronization-barrier (barrier (1+ +n-threads+))
      (let* ((n1 (floor +n-threads+ 3))
             (n2 (- +n-threads+ n1)))
        (loop for i from 1 to n1 do (spawn () (enter-barrier barrier)))
        (dotimes (i n2)
          (let ((child (spawn () (enter-barrier barrier))))
            (when (<= (random 1.0) 0.3)
              (thread-yield))
            (let ((target (if (<= (random 1.0) 0.5)
                              child
                              (random-elt (spawned-threads)))))
              (kill-thread target))))
        (mapc #'kill-thread (spawned-threads))
        (dolist (thread (spawned-threads))
          (join-thread thread :default nil))
        (barrier-count barrier)))
  0)

#+nil
(define-synchronization-test enter-barrier+unwinding.due-to-type-error.1
    (with-synchronization-barrier (barrier most-positive-fixnum)
      (setf (barrier-count barrier) +most-positive-word+)
      (handler-case (enter-barrier barrier)
        (type-error ()
          (= (barrier-count barrier) most-positive-fixnum))))
  t)

;;; Test ENTER-BARRIER's :IF-BROKEN key parameter.
(define-synchronization-test enter-barrier.1
    (flet ((test (&key if-broken)
             (with-synchronization-barrier (barrier 2)
               (let ((tester
                      (spawn ()
                        (signals-condition-p 'broken-barrier
                          (enter-barrier barrier :if-broken if-broken)))))
                 (thread-yield)
                 (sleep 0.5)
                 (break-barrier barrier)
                 (join-thread tester)))))
      (values
       (test :if-broken :error)
       (test :if-broken nil)
       (test :if-broken :punt)))
  t
  t
  nil)

;;; Test that trying to enter into an already broken barrier still
;;; results in an BROKEN-BARRIER error.
(define-synchronization-test enter-barrier.2
    (with-synchronization-barrier (barrier 0)
      (assert (broken-p (break-barrier barrier)))
      (signals-condition-p 'broken-barrier
        (enter-barrier barrier)))
  t)

;;; Test ENTER-BARRIER's :IF-STALE key parameter.
(define-synchronization-test enter-barrier.3
    (with-synchronization-barrier (barrier 0)
      (assert (finished-p (enter-barrier barrier)))
      (values (signals-condition-p 'stale-barrier
                (enter-barrier barrier :if-stale :error))
              (signals-condition-p 'stale-barrier
                (enter-barrier barrier :if-stale nil))
              (enter-barrier barrier :if-stale :punt)))
  t
  t
  nil)

;;; Assure that we can break the barrier we're currently waiting on
;;; from within a deadline handler.
(define-synchronization-test break-barrier.1
    (with-synchronization-barrier (barrier 2)
      (signals-condition-p 'broken-barrier
        (handler-bind ((sb-sys:deadline-timeout
                        #'(lambda (c)
                            (break-barrier barrier)
                            (ignore-deadline c))))
          (sb-sys:with-deadline (:seconds 0.1)
            (enter-barrier barrier)))))
  t)


;;; Initialize a barrier with a threshold of zero, and compute its
;;; needed size dynamically.
(define-synchronization-test increment-barrier-threshold.1
    (let ((n +n-threads+))
      (with-synchronization-barriers ((after-increment n)
                                      (test-barrier 0))
        (dotimes (i n)
          (spawn ()
            (increment-barrier-threshold test-barrier)
            (enter-barrier after-increment)
            (signals-condition-p 'sb-sys:deadline-timeout
              (sb-sys:with-deadline (:seconds 1.0)
                (enter-barrier test-barrier)))))
        (none #'boolean-true (spawned-threads) :key #'join-thread)))
  t)

;;; Very important test: We assure that INCREMENT-BARRIER-THRESHOLD
;;; works within a deadline handler on the same barrier as the
;;; deadline interrupted. Notice how INCREMENT-BARRIER-THRESHOLD will
;;; consequently try to recursively obtain the mutex that the thread
;;; is already holding on!
(define-synchronization-test increment-barrier-threshold.2
    (let ((n (floor +n-threads+ 2)))
      (with-synchronization-barriers ((main-barrier        (1+ n))
                                      (new-threads-barrier (1+ n)))
        (dotimes (i n)
          (named-spawn tester ()
            (handler-bind
                ((sb-sys:deadline-timeout
                  (let ((again? nil))
                    #'(lambda (c)
                        ;; SBCL < 1.0.36 contained a bug in
                        ;; CONDITION-WAIT that signalled a deadline
                        ;; twice in a row.
                        (when again? (return-from tester nil))
                        (setq again? t)
                        (increment-barrier-threshold main-barrier)
                        (spawn ()
                          (enter-barrier new-threads-barrier)
                          (enter-barrier main-barrier)
                          t)
                        (ignore-deadline c)))))
              (sb-sys:with-deadline (:seconds 0.1)
                (enter-barrier main-barrier)
                t))))
        (enter-barrier new-threads-barrier)
        (enter-barrier main-barrier)
        (all #'boolean-true (spawned-threads) :key #'join-thread)))
  t)


;;; Test that INCREMENT/DECREMENT-BARRIER-THRESHOLD do not step onto
;;; each other's toes.
(define-synchronization-test decrement-barrier-threshold.1
    (macrolet ((retry-on-signal (condition &body forms)
                 `(tagbody
                   :retry
                     (handler-case (progn ,@forms)
                       (,condition ()
                         (sleep 0.01)
                         (go :retry))))))
      (let ((n +n-threads+))
        (with-synchronization-barrier (barrier n)
          (dotimes (i n)
            (spawn ((i i))
              (let ((delta (random 25)))
                (cond ((oddp i)
                       (increment-barrier-threshold barrier delta)
                       (sleep (random 0.05))
                       (retry-on-signal invalid-threshold-delta
                         (decrement-barrier-threshold barrier delta)))
                      (t
                       (retry-on-signal invalid-threshold-delta
                         (decrement-barrier-threshold barrier delta))
                       (sleep (random 0.05))
                       (increment-barrier-threshold barrier delta))))))
          (mapc #'join-thread (spawned-threads))
          (values
           (= (barrier-threshold barrier) n)
           (= (barrier-count barrier)     0)))))
  t
  t)

;;; Assure that DECREMENT-BARRIER-THRESHOLD wakes up waiting threads
;;; if it decreases a barrier's threshold below the number of waiters.
(define-synchronization-test decrement-barrier-threshold.2
    (let ((n +n-threads+))
      (with-synchronization-barriers ((barrier (1+ n))
                                      (waiters-ready (1+ n)))
        (dotimes (i n)
          (spawn ()
            (enter-barrier waiters-ready)
            (enter-barrier barrier)
            t))
        (enter-barrier waiters-ready)
        (thread-yield)
        (sleep 0.1)
        (decrement-barrier-threshold barrier)
        (all #'boolean-true (spawned-threads) :key #'join-thread)))
  t)

;;; Similiar to above, but assure that DECREMENT-BARRIER-THRESHOLD can
;;; wake up a barrier even from within a deadline handler -- a barrier
;;; the deadline interrupted.
(define-synchronization-test decrement-barrier-threshold.3
    (with-synchronization-barrier (barrier 2)
      (handler-bind ((sb-sys:deadline-timeout
                      #'(lambda (c)
                          (decrement-barrier-threshold barrier)
                          (ignore-deadline c))))
        (sb-sys:with-deadline (:seconds 0.1)
          (enter-barrier barrier)
          t)))
  t)

;;; Assure that decrementing a barrier's threshold cannot become
;;; negative.
(define-synchronization-test decrement-barrier-threshold.4
    (with-synchronization-barrier (barrier 0)
      (signals-condition-p 'error
        (decrement-barrier-threshold barrier)))
  t)


;;; Create a cyclic-barrier, let it finish once; then reset it to new
;;; threshold, and assure that it works again.
(define-synchronization-test reset-cyclic-barrier.1
    (let* ((n +n-threads+)
           (n/2 (floor n 2)))
      (with-synchronization-barrier (barrier n :recyclable t)
        (dotimes (i n)
          (spawn () (enter-barrier barrier)))
        (mapc #'join-thread (spawned-threads))
        (reset-cyclic-barrier barrier n/2)
        (dotimes (i n/2)
          (spawn () (enter-barrier barrier)))
        (mapc #'join-thread (spawned-threads))
        (cyclic-barrier-reset-counter barrier)))
  1)


;;;; Gates

;;; Create N threads waiting until a gate is opened, then open that
;;; gate and assure that all waiters were waked up.
(define-synchronization-test gate.1
    (with-buckets collect-into ((bucket :synchronized t))
      (let ((gate (make-gate)))
        (dotimes (i +n-threads+)
          (spawn ((i i))
            (wait-open-gate gate)
            (collect-into bucket i)))
        (sleep 0.3)
        (let ((old-bucket bucket))
          (open-gate gate)
          (mapc #'join-thread (spawned-threads))
          (values old-bucket
                  (equal (sort bucket #'<) (iota +n-threads+))))))
  nil
  t)

;;; Assure that CLOSE-GATE can close a gate while other threads are
;;; operating through that gate. In particular, assure that no
;;; operation is performed once the gate is closed.
(define-synchronization-test gate.2
    (with-buckets collect-into ((bucket :synchronized t))
      (let ((gate (make-gate)))
        (dotimes (i +n-threads+)
          (spawn ((i i))
            (wait-open-gate gate)
            (when (<= (random 1.0) 0.8) (sleep (random 1.0)))
            (wait-open-gate gate)
            (collect-into bucket i)))
        (open-gate gate)
        (thread-yield)
        (close-gate gate)
        ;; sleep so threads which are pending between the second
        ;; WAIT-OPEN-GATE and the COLLECT-INTO, have a chance to
        ;; update BUCKET before we take a copy of it.
        (sleep 0.3)
        (let ((b1 (copy-list bucket))
              (b2 (progn (sleep 0.5) (copy-list bucket))))
          (open-gate gate)
          (mapc #'join-thread (spawned-threads))
          (values (equal b1 b2)
                  (equal b1 bucket)
                  (= (length bucket) +n-threads+)))))
  t
  nil
  t)

;;; Assures that WAIT-OPEN-GATE can be interrupted by deadlines.
(define-synchronization-test gates+deadlines.1
    (let* ((gate (make-gate))
           (waiter (named-spawn waiter ()
                     (handler-bind ((sb-sys:deadline-timeout
                                     #'(lambda (c)
                                         (return-from waiter :deadline))))
                       (sb-sys:with-deadline (:seconds 0.1)
                         (wait-open-gate gate)
                         t)))))
      (join-thread waiter))
  :deadline)

;;; Assure that WAIT-OPEN-GATE can be interrupted by deadlines,
;;; and resumed from the deadline handler.
(define-synchronization-test gates+deadlines.2
    (with-synchronization-barrier (ready 2)
      (let* ((gate (make-gate))
             (waiter (spawn ()
                       (handler-bind ((sb-sys:deadline-timeout
                                       #'(lambda (c)
                                           (enter-barrier ready)
                                           (ignore-deadline c))))
                         (sb-sys:with-deadline (:seconds 0.1)
                           (wait-open-gate gate)
                           t)))))
        (enter-barrier ready)
        (open-gate gate)
        (join-thread waiter)))
  t)

