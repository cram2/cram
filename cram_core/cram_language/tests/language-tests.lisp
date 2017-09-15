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

(in-package :cpl-tests)

(def-suite language-tests :in language)

(in-suite language-tests)

(define-cram-test top-level-terminates
    "FIXME" ()
  (is-true (top-level t)))

(define-cram-test par-terminates
    "FIXME" ()
  (let ((v (vector nil nil)))
    (top-level
      (par (setf (elt v 0) t)
           (setf (elt v 1) t)))
    (is (every #'identity v))))

(define-cram-test par-performs-parallel
    "FIXME" ()
  (let ((ping-pong (make-fluent :name :ping-pong :value nil)))
    (top-level
      (par (seq
             (setf (value ping-pong) :ping)
             (wait-for (fl-eq ping-pong :pong)))
           (seq
             (wait-for (fl-eq ping-pong :ping))
             (setf (value ping-pong) :pong))))
    (pass)))

(define-cram-test par-failure-handling
    "FIXME" ()
  (let ((failure nil)
        (worker-terminated nil))
    (handler-case
        (top-level
          (par (seq (sleep* 0.01) (fail))
               (seq (sleep* 1.0)  (setf worker-terminated t))))
      (plan-failure (e)
        (setf failure e)))
    (is (not worker-terminated))))

;; (test test-with-tags
;;   (for-all ()
;;     (let ((finished (make-array 2 :initial-element nil)))
;;       (with-deadlock-handling (:timeout 0.15)
;;           (top-level
;;             (with-tags
;;               (:tag task-1
;;                     (setf (aref finished 0) t))
;;               (:tag task-2
;;                     (setf (aref finished 1) t))))
;;         (is (every #'identity finished))))))

(define-cram-test par-failure-final-status-of-tasks
    "FIXME" ()
  (let ((p) (w1) (w2) (w3))
    (signals plan-failure
      (top-level
        (with-tags
          (setf p producer)
          (setf w1 worker-1)
          (setf w2 worker-2)
          (setf w3 worker-3)
          (par (:tag producer
                 (sleep* 0.1)
                 (fail))
               (:tag worker-1
                 (par (:tag worker-2
                        (sleep* 10))
                      (:tag worker-3
                        (sleep* 10))))))))
    (wait-until (become +dead+) p w1 w2 w3)
    (has-status p :failed)
    (has-status w1 :evaporated)
    (has-status w2 :evaporated)
    (has-status w3 :evaporated)))

(define-cram-test with-tags.1
    "Make sure that :TAG-created tasks do not introduce hidden
     parallelism."
    ()
  (let ((+sleep+ 0.01)
        (bag '()))
    (top-level
      (with-tags
        (seq (:tag A (push 'A bag) (sleep* (random +sleep+)))
             (:tag B (push 'B bag) (sleep* (random +sleep+)))
             (:tag C (push 'C bag) (sleep* (random +sleep+)))
             (:tag D (push 'D bag) (sleep* (random +sleep+)))
             (:tag E (push 'E bag) (sleep* (random +sleep+))))))
    (is (equal '(A B C D E) (nreverse bag)))))

(define-cram-test with-task-suspended.1
    "Basics. Test suspending, then wakeup." ()
  (top-level 
    (with-tags
      (par (:tag slave (sleep* 0.2))
           (seq (with-task-suspended (slave)
                  (wait-until (become :suspended) slave))
                (wait-until (become :running) slave)))))
  (pass))

(define-cram-test with-task-suspended.2
    "Ensure that WITH-TASK-SUSPENDED suspends and wakeup a whole tree
     of tasks." ()
  (let ((knob (fluent nil))
        (result))
    (top-level
      (with-tags
        (par (:tag waiters
               (pursue (:tag winner (seq (wait-for knob) :A))
                       (seq (wait-for (end-of-time)) :B)
                       (seq (wait-for (end-of-time)) :C)))
             (with-task-suspended (waiters)
               (setf (value knob) t)))
        (setq result (result winner))))
    (is (eq :A result))))

(define-cram-test with-task-suspended.3
    "Again ensure that WITH-TASK-SUSPENDED works on tree of tasks." ()
  (let ((knob (fluent nil))
        (counter 0)
        (aux))
    (flet ((wait ()
             (wait-for knob)
             ;; INCF is no problem as we don't care for its exact
             ;; working.
             (incf counter)))
      (top-level 
        (with-tags
          (par (:tag waiters (par (wait) (par (wait) (wait) (wait)) (wait)))
               (with-task-suspended (waiters)
                 (setf (value knob) t)
                 (sleep 0.1)
                 (setq aux counter))))))
    (is (= 0 aux))))

(define-cram-test with-task-suspended.4
    "FIXME" ()
  (flet ((time-equal (t1 t2  &key (threshold 0.01))
           (<  (/ (abs (- t2 t1))
                  internal-time-units-per-second)
               threshold)))
    (let ((final-times (make-array 2))
          (blockee-ready    (fluent nil))
          (blocker-finished (fluent nil)))
      (top-level
        (with-tags
          (par (:tag blocker
                 (let ((start-time nil))
                   (wait-for blockee-ready)
                   (with-task-suspended (blockee)
                     (setf start-time (get-internal-real-time))
                     (setf (value blocker-finished) t)
                     (sleep* 0.1)
                     (setf (aref final-times 0) (- (get-internal-real-time) start-time)))))
               (:tag blockee
                 (let ((start-time nil))
                   (on-suspension (setf start-time (get-internal-real-time))
                     (setf (value blockee-ready) t)
                     (wait-for blocker-finished))
                   (setf (aref final-times 1) (- (get-internal-real-time) start-time)))))))
      (is (>= (aref final-times 1) (aref final-times 0))))))

;;; Deprecated! We now have new semantics for suspend-protect
;; (test test-suspend-protect
;;   (flet ((time-equal (t1 t2  &key (threshold 0.01))
;;            (<=  (/ (abs (- t2 t1))
;;                    internal-time-units-per-second)
;;                 threshold)))
;;     (for-all ()
;;       (let ((protected nil)
;;             (runs 0))
;;       (with-deadlock-handling (:timeout 0.30)
;;             (top-level
;;               (with-tags
;;                 (par
;;                   (:tag blocker
;;                     (wait-for (not (eq (status blockee) :created)))
;;                     (sleep* 0.05)
;;                     (with-task-suspended blockee
;;                       (sleep* 0.05)))
;;                   (:tag blockee
;;                     (suspend-protect
;;                         (progn
;;                           (incf runs)
;;                           (sleep* 0.15))
;;                       (setf protected t))))))
;;           (is (eq protected t))
;;           (is (eql runs 2)))))))

(define-cram-test pursue.1
    "Make sure that PURSUE evaporates still-running childs if another
     child succeeds." ()
  (let ((w1) (w1-terminated nil)
        (w2) (w2-terminated nil))
    (top-level
      (with-tags
        (setq w1 worker-1)
        (setq w2 worker-2)
        (pursue (:tag worker-1
                  (sleep* 0.2)
                  (setf w1-terminated t))
                (:tag worker-2
                  (sleep* 0.05)
                  (setf w2-terminated t)))))
    (wait-until (become +dead+) w1 w2)
    (is (eq 'nil w1-terminated))
    (is (eq 't   w2-terminated))
    (has-status w1 :evaporated)
    (has-status w2 :succeeded)))

(define-cram-test pursue.2
    "Make sure that PURSUE evaporates still-running childs if another
     child fails. Also make sure that the failure propagates." ()
  (let ((w1) (w2))
    (signals plan-failure
      (top-level
        (with-tags
          (setq w1 worker-1)
          (setq w2 worker-2)
          (pursue (:tag worker-1 (sleep* 0.2))
                  (:tag worker-2 (sleep* 0.05) (fail))))))
    (wait-until (become +dead+) w1 w2)
    (has-status w1 :evaporated)
    (has-status w2 :failed)))

(define-cram-test pursue.3
    "Test corner case: WINNER and LOSER wake up simulaneosly, the
     former succeeds, the latter fails."
    ((:timeout 2.0))
  (let ((w) (l) (s)
        (sem (make-semaphore)))
    (top-level
      (with-tags
        (setq w winner)
        (setq l loser)
        (setq s signaller)
        (ignore-some-conditions (plan-failure)
          (pursue (:tag winner (wait-on-semaphore sem))
                  (:tag loser  (wait-on-semaphore sem) (fail))
                  (:tag signaller                  
                    (sleep* 0.05)
                    (signal-semaphore sem 2)
                    (sleep* 1.0))))))
    (wait-until (become +dead+) w l s)
    ;; Even if W1 succeeds, before W2 gets properly evaporated, it may
    ;; already have changed its status to :failed.
    (is (member (value (status w)) '(:succeeded :evaporated)))
    (is (member (value (status l)) '(:failed :evaporated)))
    (is (eq (value (status s)) :evaporated))))

(define-cram-test with-failure-handling.1
    "Ensure that WITH-FAILURE-HANDLING can catch normal CL errors." ()
  (let ((got-it? nil))
    (top-level 
      (pursue (with-failure-handling ((error (e)
                                        (setq got-it? e)
                                        (return)))
                (error "foof"))
              (sleep* 1.0)))
    (is-true (typep got-it? 'error))))

(define-cram-test with-failure-handling.2
    "Ensure that WITH-FAILURE-HANDLING can catch plan failures." ()
  (top-level 
    (pursue (with-failure-handling ((plan-failure (e) (return e)))
              (fail "foof"))
            (sleep* 1.0)))
  (pass))

(define-cram-test with-failure-handling.3
    "Ensure that WITH-FAILURE-HANDLING does not catch plan failures as
     normal CL errors." ()
  (signals plan-failure
    (top-level 
      (pursue (with-failure-handling ((error (e) (return e)))
                (fail "foof"))
              (sleep* 1.0)))))

(define-cram-test with-failure-handling.4
    "Ensure that a throw (and unhandled) CL error results in a CL error thrown by the top-level form" ()
  (let ((*debug-on-lisp-errors* nil))
    (signals error
      (top-level
        (pursue (par
                  (sleep* 1)
                  (par
                    (sleep* 1)
                    (error "foof")))
                (sleep* 1.0))))))

(define-cram-test with-failure-handling.5
    "Ensure that CL errors can be thrown accross task boundaries and
    WITH-FAILURE-HANDLING can catch them." ()
  (let ((got-it? nil)
        (*debug-on-lisp-errors* nil))
    (top-level
      (pursue (with-failure-handling ((error (e)
                                        (setq got-it? e)
                                        (return)))
                (par
                  (sleep* 1)
                  (par
                    (sleep* 1)
                    (error "foof"))))
              (sleep* 1.0)))
    (is-true (typep got-it? 'error))))

(define-cram-test try-all.1
    "Basic workingness of TRY-ALL. W1 succeeds, W2 must hence get
     evaporated." ()
  (let ((w1) (w1-finished nil)
        (w2) (w2-finished nil))
    (top-level
      (with-tags
        (setq w1 worker-1)
        (setq w2 worker-2)
        (try-all
          (:tag worker-1 (setf w1-finished t))
          (:tag worker-2
            (seq
              (sleep* 0.5)
              (setf w2-finished t))))))
    (is (eq 't   w1-finished))
    (is (eq 'nil w2-finished))
    (wait-until (become +dead+) w1 w2)
    (has-status w1 :succeeded)
    (has-status w2 :evaporated)))

(define-cram-test try-all.2
    "The other case; all childs fail, so TRY-ALL must propagate the
     failure." ()
  (let ((worker-1-exec nil)
        (worker-2-exec nil)
        (worker-3-exec nil)
        (err nil))
    (with-failure-handling
        ((plan-failure (e)
           (setf err e)
           (return nil)))
      (top-level
        (try-all
          (and (setf worker-1-exec t) (fail))
          (and (setf worker-2-exec t) (fail))
          (and (setf worker-3-exec t) (fail)))))
    (is (not (null worker-1-exec)))
    (is (not (null worker-2-exec)))
    (is (not (null worker-3-exec)))
    (is (typep err 'composite-failure))))

(define-cram-test try-in-order.1
    "Basic workingness of TRY-IN-ORDER. First ones fail, last one
     succeeds." ()
  (let ((bag '()))
    (top-level
      (try-in-order
        (seq (push 'A bag) (fail) (push 1 bag))
        (seq (push 'B bag) (fail) (push 2 bag))
        (seq (push 'C bag) (fail) (push 3 bag))
        (seq (push 'D bag)        (push 4 bag))))
    (is (equal '(A B C D 4) (nreverse bag)))))

(define-cram-test try-in-order.2
    "The other case: all fail, so make sure the failure propagates
     upwards." ()
  (let ((worker-1-exec nil)
        (worker-2-exec nil)
        (worker-3-exec nil))
    (signals composite-failure
      (top-level
        (try-in-order
          (and (setf worker-1-exec t) (fail))
          (and (setf worker-2-exec t) (fail))
          (and (setf worker-3-exec t) (fail)))))
    (is (eq 't worker-1-exec))
    (is (eq 't worker-2-exec))
    (is (eq 't worker-3-exec))))

(define-cram-test try-each-in-order.1
  "Success on first iteration." ()
  (let ((value (top-level
                 (try-each-in-order (value '(1 2 3))
                   value))))
    (is (eq value 1))))

(define-cram-test try-each-in-order.2
  "Success on second iteration." ()
  (let ((value (top-level
                 (try-each-in-order (value '(1 2 3))
                   (unless (evenp value)
                     (fail))
                   value))))
    (is (eq value 2))))

(define-cram-test try-each-in-order.3
    "Failure with compound failure" ()
  (let ((values (list)))
    (signals composite-failure
      (top-level
        (try-each-in-order (value '(1 2 3))
          (push value values)
          (fail))))
    (is (member 1 values))
    (is (member 2 values))
    (is (member 3 values))))

(define-cram-test with-parallel-childs.1
    "Make sure that WITH-PARALLEL-CHILDS' watcher-body is executed
     once and only once per status change." ()
  (let ((n-watches 0))
    (hangs
      (top-level
        (cpl-impl::with-parallel-childs nil (running done failed)
            ((sleep* 0.01))
          (declare (ignore running done failed))
          (incf n-watches))))
    (is (= 2 n-watches))))

(define-cram-test partial-order.1
    "Check the simplest case of a partial ordering. One task directly
     depends on another task. It must not execute before the first task
     has terminated." ()
  (let ((t-1 nil)
        (t-2 nil)
        (+sleep+ 0.1))
    (top-level
      (with-tags
        (partial-order
            ((:tag worker-1
               (sleep* (random +sleep+))
               (setf t-1 (get-internal-real-time)))
             (:tag worker-2
               (sleep* (random +sleep+))
               (setf t-2 (get-internal-real-time))))
          (:order worker-1 worker-2))))
    (is (<= t-1 t-2))))

(define-cram-test partial-order.2
    "Checks the sequential ordering of more than two tasks." ()
  (let ((t-1 nil)
        (t-2 nil)
        (t-3 nil)
        (+sleep+ 0.1))
    (top-level
      (with-tags
        (partial-order
            ((:tag worker-1
               (sleep* (random +sleep+))
               (setf t-1 (get-internal-real-time)))
             (:tag worker-2
               (sleep* (random +sleep+))
               (setf t-2 (get-internal-real-time)))
             (:tag worker-3
               (sleep* (random +sleep+))
               (setf t-3 (get-internal-real-time))))
          (:order worker-1 worker-3)
          (:order worker-3 worker-2))))
    (is (<= t-1 t-3))
    (is (<= t-3 t-2))))

(define-cram-test partial-order.3
    "Tasks 1 and 2 execute in paralell, task 3 is constrained by both,
    i.e. it must not run before both terminated." ()
  (let ((t-1 nil)
        (t-2 nil)
        (t-3 nil)
        (+sleep+ 0.1))
    (top-level
      (with-tags
        (partial-order
            ((:tag worker-1
               (sleep* (random +sleep+))
               (setf t-1 (get-internal-real-time)))
             (:tag worker-2
               (sleep* (random +sleep+))
               (setf t-2 (get-internal-real-time)))
             (:tag worker-3
               (sleep* (random +sleep+))
               (setf t-3 (get-internal-real-time))))
          (:order worker-1 worker-3)
          (:order worker-2 worker-3))))
    (is (<= t-1 t-3))
    (is (<= t-2 t-3))))

(define-cram-test partial-order.4
    "Ring dependencies in orderings will always lead to deadlocks."
    ((:n-runs 10) (:timeout nil))
  (hangs
    (top-level
      (with-tags
        (partial-order
            ((:tag worker-1)
             (:tag worker-2))
          (:order worker-1 worker-2)
          (:order worker-2 worker-1))))))


(define-cram-test misc.1
    "Check that after wakeup deadline will be active again." ()
  (let ((fluent (fluent nil)))
    (top-level
      (with-tags
        (par (:tag waiter (wait-for fluent) :A)
             (:tag controller
               (sleep* 0.1)
               (suspend-tasks waiter)
               (sleep* 0.1)
               (wake-up-tasks waiter)
               (sleep* 0.1)
               (evaporate-tasks waiter)
               (sleep* 1.0)
               :B))
        (wait-until (become +dead+) waiter controller)))
    (pass)))

(define-cram-test failure.1
    "Test that failures are propagated." ()
  (let ((knob (fluent nil)))
    (signals plan-failure
      (top-level 
        (with-tags
          (par
            (:tag slave 
              (pursue (seq (wait-for knob) (fail))
                      (seq (sleep* 3.0) :B)
                      (seq (sleep* 3.0) :C)))
            (with-task-suspended (slave)
              (setf (value knob) t))))))))
