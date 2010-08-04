
(in-package :kipla-reasoning)

;;(defparameter +et-path+ "/home/demmeln/work/et/mac/grasp-cluster-1.ek")
(defparameter +et-path+ "/home/demmeln/work/et/mac/demo-traces/3-COMPLETE-MISSING-OBJS.ek")

(progn
  (import 'kipla::achieve :kipla-reasoning)
  (import 'kipla::object-in-hand :kipla-reasoning)
  (import 'kipla::object-placed-at :kipla-reasoning)
;;  (import 'kipla::loc :kipla-reasoning)
  (import 'kipla::robot :kipla-reasoning)
  (import 'kipla::perceive :kipla-reasoning)
  (import 'kipla::perceive-all :kipla-reasoning)
;;  (import 'kipla::?obj :kipla-reasoning)
;;  (import 'kipla::?side :kipla-reasoning)
  (import 'kipla::owl-type :kipla-reasoning)
  (import 'kipla::cowsmilk-product :kipla-reasoning)
  (import 'kipla::shape :kipla-reasoning)
  (import 'kipla::arms-at :kipla-reasoning)
;;  (import 'kipla::?traj :kipla-reasoning)
  (import 'kipla::looking-at :kipla-reasoning)
;;  (import 'kipla::?lo :kipla-reasoning)
  (import 'kipla::arm-parked :kipla-reasoning)
;;  (import 'kipla::?loc :kipla-reasoning)
  (setf cpl-impl::*task-tree-print-children* nil)
  (setf cpl-impl::*task-tree-print-path* :one)
  (eval-when (:load-toplevel)
    (setf cet:*episode-knowledge* (cet:load-episode-knowledge +et-path+))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; BASIC QUERIES
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun demmeln-query-tasks ()
  (force-ll (prolog '(task ?task))))

(defun demmeln-query-succeeded-tasks ()
  (force-ll (prolog '(and
                      (task ?task)
                      (task-goal ?task ?goal)
                      (task-outcome ?task :succeeded)))))

(defun demmeln-query-failed-tasks ()
  (force-ll (prolog '(and
                      (task ?task)
                      (task-goal ?task ?goal)
                      (task-outcome ?task :failed)))))

(defun demmeln-query-evaporated-tasks ()
  (force-ll (prolog '(and
                      (task ?task)
                      (task-goal ?task ?goal)
                      (task-outcome ?task :evaporated)))))

(defun demmeln-query-main-goals ()
  (force-ll (prolog '(filter-bindings (?goal ?outcome)
                      (top-level ?top-level)
                      (subtask ?top-level ?task)
                      (task-goal ?task ?goal)
                      (task-outcome ?task ?outcome)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; KNOWLEDGE BASE (TAXONOMY) / PLAN PARAMETERIZATION
;;;
;;; * Did the robot grasp an object which was a drink?
;;; * If yes, what kind of drink?
;;; * How did the robot grip the object?
;;; * Which hand did the robot use?
;;; * Where was the object, when it was held by the robot? (TODO)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun is-food-or-drink (desig-type)
  (eq 'cowsmilk-product desig-type)
  #+nil(force-ll (json-prolog:prolog
             `(owl-subclass-of ,(desig-type->owl-type-hack desig-type) "knowrob:'FoodOrDrink'"))))

(defun desig-type->owl-type-hack (desig-type)
  (case desig-type
    (cowsmilk-product "knowrob:'CowsMilk-Product'")))

(defun demmeln-query-knowledge-base-1 ()
  (force-ll (prolog '(and
                      (task ?task)
                      (task-goal ?task (achieve (object-in-hand ?object ?side)))
                      (desig-prop ?object (owl-type ?owl-type))
                      (lisp-pred is-food-or-drink ?owl-type))))
    ;; wie bekommen ich object typ?
    ;; (force-ll (json-prolog:prolog `("owl_subclass_of" ?A  "knowrob:'FoodOrDrink'")))
    ;; wie bekommen ich den grasp?
    ;; evt jlo anfrage zur den koordinaten?
    )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; ASSERTING INTENTION - from iros paper
;;;
;;; * In order to grasp an object, did the robot move?
;;; * Where?
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun demmeln-query-intention-1 ()
  (force-ll (prolog '(filter-bindings (?loc ?task)
                      (task ?task)
                      (task-goal ?task (achieve (kipla::loc robot ?loc)))
                      (subtask+ ?super ?task) ;; here you can try subtask vs subtask+
                      (task-goal ?super (achieve (object-in-hand ?object ?side)))))))

(defun demmeln-query-intention-2 ()
  (force-ll (prolog '(filter-bindings (?task ?loc ?loc-to-what ?loc-with-what)
                      (task ?task)
                      (task-goal ?task (achieve (kipla::loc robot ?loc)))
                      (desig-prop ?loc (to ?loc-to-what))
                      (desig-prop ?loc (obj ?loc-with-what))
                      (subtask+ ?super ?task)
                      (task-goal ?super (achieve (object-in-hand ?object ?side)))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; TASK STRUCTURE
;;;
;;; * What did we (try to) achieve?
;;; * Which objects did we handle?
;;; * Which objects did we percieve?
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun demmeln-query-achievements ()
  (force-ll (prolog '(filter-bindings (?achieve ?outcome)
                      (task-goal ?task (achieve . ?achieve))
                      (task-outcome ?task ?outcome)))))
(defun demmeln-query-achievements-objects ()
  (force-ll (prolog '(filter-bindings (?achieve ?object ?outcome)
                      (task-goal ?task (achieve ?achieve))
                      (task-outcome ?task ?outcome)
                      (or (== ?achieve (?_ ?_ ?desig . ?_)) ;; could restrict type
                          (== ?achieve (?_ ?desig . ?_)))   ;; of actions further
                      (instance-of designator ?desig)
                      (desig-prop ?desig (obj ?object))))))
(defun demmeln-query-perceptions ()
  (force-ll (prolog '(filter-bindings (?object ?outcome)
                      (task-goal ?task (perceive ?object))
                      (task-outcome ?task ?outcome)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; FAILURES
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun demmeln-query-failed-tasks-siblings ()
  (force-ll (prolog '(filter-bindings (?failed-task ?sibling-list)
                      (task ?failed-task)
                      (task-outcome ?failed-task :failed)
                      (findall (?sibling)
                       (task-sibling ?failed-task ?sibling)
                       ?sibling-list)))))
(defun demmeln-query-failed-tasks-siblings-after ()
  (force-ll (prolog '(filter-bindings (?failed-task ?failed-goal ?sibling-list)
                      (task ?failed-task)
                      (task-outcome ?failed-task :failed)
                      (task-ended-at ?failed-task ?failed-end-time)
                      (task-goal ?failed-task ?failed-goal)
                      (findall (:task ?sibling :goal ?sibling-goal)
                       (and (task-sibling ?failed-task ?sibling)
                        (task-created-at ?sibling ?sibling-create-time)
                        (>= ?sibling-create-time ?failed-end-time)
                        (task-goal ?sibling ?sibling-goal))
                       ?sibling-list)))))
(defun demmeln-query-failed-tasks-retries ()
  (force-ll (prolog '(filter-bindings (?failed-task ?failed-goal ?sibling ?sibling-outcome)
                      (task ?failed-task)
                      (task-outcome ?failed-task :failed)
                      (task-goal ?failed-task ?failed-goal)                       
                      (task-sibling ?failed-task ?sibling)
                      (task-goal ?sibling ?failed-goal)
                      (task-outcome ?sibling ?sibling-outcome)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; UNACHIEVED-GOAL-ERROR
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; not yet
(defun demmeln-query-unachieved-goal-error ()
  (force-ll (prolog '(and
                      (top-level ?top-level)
                      (subtask ?top-level ?task)
                      (task-goal ?task ?goal)
                      (task-ended-at ?task ?time)
                      ;; how can i access rete believes? (TODO)
                      (not (holds ()))))))