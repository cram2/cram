(in-package :cpl-tests)

(def-suite task-tests :in language)

(in-suite task-tests)

;;; Child succeeds ==> parent unaffected

(define-cram-test tasks--basics.1
    "Child awakes before parent." ()
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep* 0.1))
       (:task B (sleep* 0.025)))
    (wait-until (become +dead+) A B)
    (has-status A :succeeded)
    (has-status B :succeeded)))

;;; Parent succeeds => child evaporates

(define-cram-test tasks--basics.2
    "Parent awakes before child." ()
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep* 0.025))
       (:task B (wait-for (end-of-time))))
    (wait-until (become +dead+) A B)
    (has-status A :succeeded)
    (has-status B :evaporated)))

(define-cram-test basics.3
    "Grant-childs awake before childs awake before parent."
    ((:timeout 10.0)
     (:n-runs  10)
     (:generators (n-Bs (gen-integer :min 1 :max 20))
                  (n-Cs (gen-integer :min 1 :max 5))))
  (with-task-hierarchy ((A  -> Bs)
                        (Bs -> Cs)
                        (Cs ->   ))      
      ((:task  A       (sleep* 0.5))
       (:tasks Bs n-Bs (sleep* 0.1))
       (:tasks Cs n-Cs (sleep* 0.001)))
    (wait-until (become +dead+) A Bs Cs)
    (has-status  A  :succeeded)
    (have-status Bs :succeeded)
    (have-status Cs :succeeded)))

;;; Parent suspends => child suspends

(define-cram-test tasks--suspend.1
    "Suspend parent, make sure child becomes suspended, too.
     Then wake up parent, make sure child awakes, too."
    ()
  (let ((knob (fluent nil)))
    (with-task-hierarchy ((A -> B)
                          (B ->  ))      
        ((:task A (wait-for knob))
         (:task B (wait-for knob)))
      (suspend-tasks A) (wait-until (become :suspended) A B)
      (setf (value knob) t)
      ;; Still suspended?
      (sleep 0.1) (have-status `(,A ,B) :suspended)
      (wake-up-tasks A) (wait-until (become +dead+) A B)
      (has-status A :succeeded)
      (has-status B :succeeded :evaporated))))

;;; Parent & child suspended, child wake-up => error

(define-cram-test tasks--suspend.2 
    "Like TASKS-SUSPEND.1, but wake up the child first.
     This violates the invariant that a child must not be running
     without its parent running, too. Hence we actually expect an
     error on the attempt to wake up."
    ((:skip "Fails as of 2010-04-11"))
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep* 10.0))
       (:task B (sleep* 10.0)))
    (suspend-tasks A) (wait-until (become :suspended) A B)
    (signals error
      (wake-up-tasks B))
    (evaporate-tasks-and-wait A B)))

;;; Child suspends => parent unaffected

(define-cram-test tasks--suspend.3
    "Suspend child, make sure parent is not affected.
     Let parent succeed, check child gets evaporated." ()
  (let ((knob (fluent nil)))
    (with-task-hierarchy ((A -> B)
                          (B ->  ))      
        ((:task A (wait-for knob))
         (:task B (wait-for knob)))
      (suspend-tasks B)  (wait-until (become :suspended) B)      
      (sleep* 0.1)
      (has-status A :running) (has-status B :suspended)
      (setf (value knob) t)
      (wait-until (become +dead+) A B)
      (has-status A :succeeded) (has-status B :evaporated))))

(define-cram-test tasks--suspend.4
    "Suspend child, make sure parent is not affected. Let parent
     succeed, and at the same time wake the child again." ()
  (let ((knob (fluent nil)))
    (with-task-hierarchy ((A -> B)
                          (B ->  ))      
        ((:task A (wait-for knob))
         (:task B (wait-for knob)))
      (suspend-tasks B) (wait-until (become :suspended) B)      
      (sleep* 0.1)
      (has-status A :running) (has-status B :suspended)
      (setf (value knob) t)
      (wake-up-tasks B)
      (wait-until (become +dead+) A B)
      (has-status A :succeeded)
      (has-status B :succeeded :evaporated))))

;;; Parent evaporates => child evaporates

(define-cram-test tasks--terminate.1
    "Evaporate parent, make sure child gets evaporated, too." ()
  (with-task-hierarchy ((A -> B)
                        (B ->  ))    
      ((:task A (sleep* 10.0))
       (:task B (sleep* 10.0)))
    (evaporate-tasks A)
    (wait-until (become :evaporated) A B)
    (pass)))

;;; Child evaporates, parent running => error

(define-cram-test tasks--terminate.2
    "This is not an error. Evaporation of a child task should not
cause the parent to be evaporated. Ignore this test."
    ((:skip "Invalid test as of 2011-02-10"))
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep* 10.0))
       (:task B (sleep* 10.0)))
    (signals error
      (evaporate-tasks B))
    (evaporate-tasks-and-wait (become :evaporated) A B)))

;;; Parent fails => child evaporates

(define-cram-test tasks--failure.1
    "Parent fails, make sure child gets evaporated." ()
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep* 0.1) (fail))
       (:task B (sleep* 10.0)))
    (wait-until (become :failed) A)
    (wait-until (become :evaporated) B)
    (pass)))

;;; Child fails => parent fails

#||
  (define-cram-test tasks--failure.2
      "Child fails, make sure parent fails, too."
      ((:skip "Fails as of 2010-04-11"))
    (with-task-hierarchy ((A -> B)
                          (B ->  ))      
        ((:task A (sleep* 10.0))
         (:task B (sleep* 10.0)))
      (terminate B :failed)
      (wait-until (become :failed) A B)
      (pass)))
||#

;;; Because failures are implemented by explicit propagating in the
;;; plan-macros, the above commented out test case can not work, and
;;; we have to use the constructs of the CRAM language:

(define-cram-test tasks--failure.2
    "Child fails, make sure parent fails, too." ()
  (let ((parent) (child1) (child2))
    (signals plan-failure
      (top-level
        (with-tags
          (setq parent P) (setq child1 C1) (setq child2 C2)
          (:tag P
            (par (:tag C1 (sleep* 0.1) (fail))
                 (:tag C2 (sleep* 10.0)))))))
    (wait-until (become +dead+) parent child1 child2)
    (has-status parent :failed)
    (has-status child1 :failed)
    (has-status child2 :evaporated)))

