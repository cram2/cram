(in-package :cpl-tests)

(def-suite task-tests :in language)

(in-suite task-tests)

;;; Child succeeds ==> parent unaffected

(define-cram-test tasks--basics.1
    "Child awakes before parent." ()
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep 0.1))
       (:task B (sleep 0.025)))
    (wait-until (become +dead+) A B)
    (has-status A :succeeded)
    (has-status B :succeeded)))

;;; Parent succeeds => child evaporates

(define-cram-test tasks--basics.2
    "Parent awakes before child." ()
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep 0.025))
       (:task B (sleep 0.05)))
    (wait-until (become +dead+) A B)
    (has-status A :succeeded)
    (has-status B :evaporated)))

;;; Parent suspends => child suspends

(define-cram-test tasks--suspend.1
    "Suspend parent, make sure child becomes suspended, too.
     Then wake up parent, make sure child awakes, too."
    ()
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep 10.0))
       (:task B (sleep 10.0)))
    (suspend A) (wait-until (become :suspended) A B)
    (wake-up A) (wait-until (become :running) A B)
    (evaporate-and-wait A B)
    (pass)))

;;; Parent & child suspended, child wake-up => error

(define-cram-test tasks--suspend.2 
    "Like TASKS-SUSPEND.1, but wake up the child first.
     This violates the invariant that a child must not be running
     without its parent running, too. Hence we actually expect an
     error on the attempt to wake up."
    ((:skip "Fails as of 2010-04-11"))
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep 10.0))
       (:task B (sleep 10.0)))
    (suspend A) (wait-until (become :suspended) A B)
    (signals error
      (wake-up B))
    (evaporate-and-wait A B)))

;;; Child suspends => parent unaffected

(define-cram-test tasks--suspend.3
    "Suspend child, make sure parent is not affected.
     Then wake up the child again." ()
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep 10.0))
       (:task B (sleep 10.0)))
    (suspend B)  (wait-until (become :suspended) B)
    (sleep 0.01) (has-status A :running)
    (wake-up B)  (wait-until (become :running) B)
    (evaporate-and-wait A B)
    (pass)))

;;; Parent evaporates => child evaporates

(define-cram-test tasks--terminate.1
    "Evaporate parent, make sure child gets evaporated, too." ()
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep 10.0))
       (:task B (sleep 10.0)))
    (evaporate A)
    (wait-until (become :evaporated) A B)
    (pass)))

;;; Child evaporates, parent running => error

(define-cram-test tasks--terminate.2
    ""
    ((:skip "Fails as of 2010-04-11"))
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep 10.0))
       (:task B (sleep 10.0)))
    (signals error
      (evaporate B))
    (evaporate-and-wait (become :evaporated) A B)))

;;; Parent fails => child evaporates

(define-cram-test tasks--failure.1
    "Parent fails, make sure child gets evaporated." ()
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep 10.0))
       (:task B (sleep 10.0)))
    (terminate A :failed)
    (wait-until (become :failed) A)
    (wait-until (become :evaporated) B)
    (pass)))

;;; Child fails => parent fails

(define-cram-test tasks--failure.2
    "Child fails, make sure parent fails, too."
    ((:skip "Fails as of 2010-04-11"))
  (with-task-hierarchy ((A -> B)
                        (B ->  ))      
      ((:task A (sleep 10.0))
       (:task B (sleep 10.0)))
    (terminate B :failed)
    (wait-until (become :failed) A B)
    (pass)))
