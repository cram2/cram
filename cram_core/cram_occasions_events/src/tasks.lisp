;;;
;;; Copyright (C) 2009 by Nikolaus Demmel <demmeln@cs.tum.edu>
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

(in-package :cram-occasions-events)

;;; Note: #demmeln: not handling code replacements in any way atm.

(defun task-status-fluent-name (task-tree-node)
  "Assume node is not stale."
  (name (task-tree-node-status-fluent task-tree-node)))

(defgeneric extract-task-error (err)
  (:method ((err plan-failure))
    err)
  (:method ((err common-lisp-error-envelope))
    (envelop-error err)))

(defun task-children (task)
  (mapcar #'cdr (task-tree-node-children task)))

(def-fact-group tasks (coe:holds)

  ;; top-level
  (<- (top-level-task ?top-level-name ?top-level-task-node)
    (bound ?top-level-name)
    (lisp-fun cpl:get-top-level-task-tree ?top-level-name ?top-level-task-node))

  (<- (top-level-episode-knowledge ?top-level-name ?top-level-episode)
    (bound ?top-level-name)
    (lisp-fun cet:get-top-level-episode-knowledge ?top-level-name ?top-level-episode))

  ;; util
  (<- (task-full-path ?task-node ?path)
    (bound ?task-node)
    (lisp-fun cpl:task-tree-node-path ?task-node ?path))

  (<- (task-from-path ?task-path ?task-node)
    (bound ?task-path)
    (top-level-name ?top-level-name)
    (coe:top-level-task ?top-level-name ?top-level-task)
    (lisp-fun cpl:task-tree-node ?task-path ?top-level-task ?task-node))
  
  ;; tasks of top-level
  (<- (task-of-top-level ?top-level-name ?task-node)
    (bound ?top-level-name)
    (top-level-task ?top-level-name ?top-level-task-node)
    ;; (lisp-fun cpl:flatten-task-tree ?top-level-task-node ?all-task-nodes)
    (lisp-fun cpl:flatten-task-tree-broad ?top-level-task-node ?all-task-nodes)
    (member ?task-node ?all-task-nodes))

  ;; task for subtree
  (<- (task ?top-level-name ?subtree-path ?task-node)
    (bound ?top-level-name)
    (bound ?subtree-path)
    (top-level-task ?top-level-name ?top-level-task)
    (lisp-fun cpl:task-tree-node ?subtree-path ?top-level-task ?subtree-task)
    ;; (lisp-fun cpl:flatten-task-tree ?subtree-task ?all-subtree-tasks)
    (lisp-fun cpl:flatten-task-tree-broad ?subtree-task ?all-subtree-tasks)
    (member ?task-node ?all-subtree-tasks))

  ;; subtask
  (<- (subtask ?task ?subtask)
    (bound ?task)
    (lisp-fun cpl:task-tree-node-children ?task ?children)
    (member (?_ . ?subtask) ?children))

  (<- (subtask ?task ?subtask)
    (not (bound ?task))
    (bound ?subtask)
    (lisp-fun cpl:task-tree-node-parent ?subtask ?task)
    (lisp-pred identity ?task))

  ;; (<- (subtask ?task ?subtask)
  ;;   (not (bound ?task))
  ;;   (not (bound ?subtask))
  ;;   (task ?task)
  ;;   (subtask ?task ?subtask))

  ;; subtask+
  (<- (subtask+ ?task ?subtask)
    (subtask ?task ?subtask))

  (<- (subtask+ ?task ?subtask)
    (subtask ?task ?tmp)
    (subtask+ ?tmp ?subtask))

  ;; task-sibling
  (<- (task-sibling ?task ?sibling)
    (bound ?task)
    (subtask ?parent ?task)
    (subtask ?parent ?sibling)
    (not (== ?sibling ?task)))

  ;; (<- (task-sibling ?task ?sibling)
  ;;   (not (bound ?task))
  ;;   (subtask ?parent ?sibling)
  ;;   (subtask ?parent ?task)
  ;;   (not (== ?sibling ?task)))

  ;; task-result
  (<- (task-result ?task ?result)
    (bound ?task)
    (lisp-fun cpl:task-tree-node-result ?task ?result))

  ;; task-parameter
  (<- (task-parameter ?task ?parameter)
    (bound ?task)
    (lisp-fun cpl:task-tree-node-parameters ?task ?parameters)
    (member ?parameter ?parameters))

  ;; task-status-fluent
  (<- (task-status-fluent ?task ?fluent)
    (bound ?task)
    (lisp-fun cpl:task-tree-node-status-fluent ?task ?fluent))

  (<- (fluent-value ?fluent ?value)
    (bound ?fluent)
    (not (equal ?fluent NIL))
    (lisp-fun cpl:value ?fluent ?value))

  (<- (task-status ?task ?status)
    (bound ?task)
    (task-status-fluent ?task ?fluent)
    (fluent-value ?fluent ?status))

  (<- (task-outcome ?task ?outcome)
    (bound ?task)
    (member ?outcome (:succeeded :failed :evaporated))
    (task-status ?task ?outcome))

  (<- (task-error ?task ?error)
    (bound ?task)
    (task-outcome ?task :failed)
    (task-result ?task ?result)
    (lisp-fun extract-task-error ?result ?error))

  ;; execution trace related
  ;; PLEASE NOTE: this only work if the episode already ended execution
  ;; Live in the middle of the episode FLUENT-DURATIONS does not work.
  (<- (coe:holds (fluent-value ?fluent ?value) ?top-level-name ?time)
    (bound ?fluent)
    (bound ?top-level-name)
    (top-level-episode-knowledge ?top-level-name ?episode)
    (lisp-pred identity ?fluent)
    (lisp-fun cpl-impl:name ?fluent ?fluent-name)
    (lisp-fun cet:episode-knowledge-fluent-durations ?fluent-name ?episode ?durations)
    (member (?value . ?duration) ?durations)
    (cram-occasions-events:duration-includes ?duration ?time))

  (<- (coe:holds (task-status ?task ?status) ?top-level-name ?time)
    (bound ?top-level-name)
    (task-status-fluent ?task ?status-fluent)
    (coe:holds (fluent-value ?status-fluent ?status) ?top-level-name ?time))

  ;; task times
  (<- (task-created-at ?top-level-name ?task ?time)
    (bound ?top-level-name)
    (bound ?task)
    (coe:holds (task-status ?task :created) ?top-level-name (coe:at ?time)))

  (<- (task-started-at ?top-level-name ?task ?time)
    (bound ?top-level-name)
    (bound ?task)
    ;; (task ?task)
    (bagof ?time
           (coe:holds (task-status ?task :running) ?top-level-name (coe:at ?time))
           ?times)
    (sort ?times < (?time . ?_)))

  (<- (task-ended-at ?top-level-name ?task ?time)
    (bound ?top-level-name)
    (bound ?task)
    ;; (task ?task)
    (member ?status (:succeeded :failed :evaporated))
    (coe:holds (task-status ?task ?status) ?top-level-name (coe:at ?time)))

  ;; task next and previous sibling
  (<- (task-next-sibling ?top-level-name ?task ?next-task)
    (bound ?top-level-name)
    (bound ?task)
    (bagof ?sibling (task-sibling ?task ?sibling) ?siblings)
    (member ?next-task ?siblings)
    (task-created-at ?top-level-name ?task ?created-time-task)
    (task-created-at ?top-level-name ?next-task ?created-time-next-task)
    (<= ?created-time-task ?created-time-next-task)
    (forall (and
             (member ?other-next-task ?siblings)
             (not (== ?next-task ?other-next-task))
             (task-created-at ?top-level-name ?other-next-task ?created-time-other-next-task)
             (<= ?created-time-task ?created-time-other-next-task))
            (<= ?created-time-next-task ?created-time-other-next-task)))

  (<- (task-previous-sibling ?top-level-name ?task ?next-task)
    (bound ?top-level-name)
    (bound ?task)
    (bagof ?sibling (task-sibling ?task ?sibling) ?siblings)
    (member ?next-task ?siblings)
    (task-created-at ?top-level-name ?task ?created-time-task)
    (task-created-at ?top-level-name ?next-task ?created-time-next-task)
    (>= ?created-time-task ?created-time-next-task)
    (forall (and
             (member ?other-next-task ?siblings)
             (not (== ?next-task ?other-next-task))
             (task-created-at ?top-level-name ?other-next-task ?created-time-other-next-task)
             (>= ?created-time-task ?created-time-other-next-task))
            (>= ?created-time-next-task ?created-time-other-next-task)))


  #+now-placed-in-cram-plan-occasions-events
  ((<- (perform-task-of-top-level ?top-level-name ?task-node)
    (bound ?top-level-name)
    (task-of-top-level ?top-level-name ?task-node)
    (lisp-fun cpl:task-tree-node-path ?task-node (?path . ?_))
    (equal ?path (cpl:goal (exe:perform ?_) . ?_)))

  (<- (perform-task ?top-level-name ?subtree-path ?task-node)
    (bound ?top-level-name)
    (bound ?subtree-path)
    (task ?top-level-name ?subtree-path ?task-node)
    (lisp-fun cpl:task-tree-node-path ?task-node (?path . ?_))
    (equal ?path (cpl:goal (exe:perform ?_) . ?_)))

  (<- (task-specific-action ?top-level-name ?subtree-path ?action-type ?task ?designator)
    (bound ?top-level-name)
    (bound ?subtree-path)
    (perform-task ?top-level-name ?subtree-path ?task)
    (task-parameter ?task ?designator)
    (lisp-type ?designator desig:action-designator)
    (desig:desig-prop ?designator (:type ?action-type)))

  (<- (task-navigating-action ?top-level-name ?subtree-path ?task ?designator)
    (task-specific-action ?top-level-name ?subtree-path :navigating ?task ?designator))

  (<- (task-fetching-action ?top-level-name ?subtree-path ?task ?designator)
    (task-specific-action ?top-level-name ?subtree-path :fetching ?task ?designator))

  (<- (task-picking-up-action ?top-level-name ?subtree-path ?task ?designator)
    (task-specific-action ?top-level-name ?subtree-path :picking-up ?task ?designator))

  (<- (task-delivering-action ?top-level-name ?subtree-path ?task ?designator)
    (task-specific-action ?top-level-name ?subtree-path :delivering ?task ?designator))

    ;; task next and previous perform action sibling
  (<- (task-next-action-sibling ?top-level-name ?subtree-path ?task ?action-type ?next-task)
    (bound ?top-level-name)
    (bound ?subtree-path)
    (bound ?task)
    ;; (bound ?action-type)
    (bagof ?sibling (task-sibling ?task ?sibling) ?siblings)
    (member ?next-task ?siblings)
    (task-specific-action ?top-level-name ?subtree-path ?action-type ?next-task ?_)
    ;; (task-created-at ?top-level-name ?task ?created-time-task)
    ;; (task-created-at ?top-level-name ?next-task ?created-time-next-task)
    ;; (<= ?created-time-task ?created-time-next-task)
    ;; (forall (and
    ;;          (member ?other-next-task ?siblings)
    ;;          (task-specific-action ?top-level-name ?subtree-path ?action-type ?other-next-task ?_)
    ;;          (not (== ?next-task ?other-next-task))
    ;;          (task-created-at ?top-level-name ?other-next-task ?created-time-other-next-task)
    ;;          (<= ?created-time-task ?created-time-other-next-task))
    ;;         (<= ?created-time-next-task ?created-time-other-next-task))
    )

  (<- (task-previous-action-sibling ?top-level-name ?subtree-path ?task ?action-type ?next-task)
    (bound ?top-level-name)
    (bound ?subtree-path)
    (bound ?task)
    ;; (bound ?action-type)
    (bagof ?sibling (task-sibling ?task ?sibling) ?siblings)
    (lisp-fun cut:force-ll ?siblings ?siblings-list)
    (lisp-fun reverse ?siblings-list ?siblings-list-reversed)
    (member ?previous-task ?siblings-list-reversed)
    (task-specific-action ?top-level-name ?subtree-path ?action-type ?previous-task ?_)
    ;; (task-created-at ?top-level-name ?task ?created-time-task)
    ;; (task-created-at ?top-level-name ?next-task ?created-time-next-task)
    ;; (>= ?created-time-task ?created-time-next-task)
    ;; (forall (and
    ;;          (member ?other-next-task ?siblings)
    ;;          (not (== ?next-task ?other-next-task))
    ;;          (task-specific-action ?top-level-name ?subtree-path ?action-type ?other-next-task ?_)
    ;;          (task-created-at ?top-level-name ?other-next-task ?created-time-other-next-task)
    ;;          (>= ?created-time-task ?created-time-other-next-task))
    ;;         (>= ?created-time-next-task ?created-time-other-next-task))
    ))
  )





  
#+the-rest-is-all-test-stuff-so-commented-it-out-to-avoid-dependency-problems
(
(defun test ()
  (cet:enable-fluent-tracing)
  (cpl-impl::remove-top-level-task-tree :top-level)

  (pr2-proj:with-simulated-robot
    (pr2-pp-demo::demo-random nil))

  (cut:force-ll (prolog:prolog `(task-navigating-action :top-level ((demo-random))
                                                        ?task ?designator))))

(defun test-next-sibling-time ()
  (cet:enable-fluent-tracing)
  (cpl-impl::remove-top-level-task-tree :top-level)

  (pr2-proj:with-simulated-robot
    (pr2-pp-demo::demo-random nil))

  (cut:force-ll
   (prolog:prolog `(and (task-navigating-action :top-level ((demo-random)) ?task ?des)
                        (task-next-sibling :top-level ?task ?next-task)
                        (task-created-at :top-level ?task ?created)
                        (task-created-at :top-level ?next-task ?next-created)
                        (format "time: ~a   time next: ~a~%" ?created ?next-created)))))

(defun test-failed-actions ()
  (cet:enable-fluent-tracing)
  (cpl-impl::remove-top-level-task-tree :top-level)

  (pr2-proj:with-simulated-robot
    (pr2-pp-demo::demo-random))

  (cut:force-ll
   (prolog:prolog `(and (task-specific-action :top-level ((demo-random)) :fetching ?task ?desig)
                        (task-outcome ?task :failed)
                        (format "desig: ~a~%" ?desig)))))

(defun test-find-location-for-pick-up-using-occasions ()
  (cet:enable-fluent-tracing)
  (cpl-impl::remove-top-level-task-tree :top-level)

  (proj:with-projection-environment pr2-proj:pr2-bullet-projection-environment
    (cpl-impl::named-top-level (:name :top-level)
      (pr2-pp-demo::demo-random nil))

    (let ((top-level-name :top-level))
      (cut:var-value
       '?pick-location
       (car
        (prolog:prolog
         `(and (task-fetching-action ,top-level-name ((demo-random)) ?fetching-task ?_)
               (task-full-path ?fetching-task ?fetching-path)
               (task-picking-up-action ,top-level-name ?fetching-path ?picking-up-task ?_)
               (task-outcome ?picking-up-task :succeeded)
               (task-started-at ,top-level-name ?picking-up-task ?picking-up-start)
               (cram-robot-interfaces:robot ?robot)
               (btr:timeline ?timeline)
               (coe:holds ?timeline (cpoe:loc ?robot ?pick-location)
                          (coe:at ?picking-up-start)))))))))
)




#+this-old-code-is-based-on-episodes-and-not-used-anymore-because-we-work-only-on-task-trees
((def-fact-group fluents (holds)
  ;; FLUENT
  (<- (fluent ?fluent)
    (not (bound ?fluent))
    (lisp-fun cet:episode-knowledge-traced-fluent-names ?fluents)
    (member ?fluent ?fluents))

  (<- (fluent ?fluent)
    ;; bit of a hack to increase performance
    (bound ?fluent)
    (lisp-pred symbolp ?fluent))

  ;; HOLDS FLUENT-VALUE
  (<- (holds (fluent-value ?fluent ?value) ?t)
    (fluent ?fluent)
    (lisp-fun cet:episode-knowledge-fluent-durations ?fluent ?durations)
    (member (?value . ?duration) ?durations)
    (duration-includes ?duration ?t))

  ;; For execution-time fluent access
  (<- (fluent-value ?fluent ?value)
    (lisp-fun value ?fluent ?value)))

(def-fact-group tasks (holds)

  ;; TOP-LEVEL
  (<- (top-level ?top-level-task)
    (lisp-fun cet:episode-knowledge-task-tree ?top-level-task))
  
  ;; TASK
  (<- (task ?task)
    (lisp-fun cet:episode-knowledge-task-list ?tasks)
    (member ?task ?tasks))

  (<- (goal-task ?task)
    (lisp-fun cet:episode-knowledge-goal-task-list ?tasks)
    (member ?task ?tasks))

  (<- (task-path ?task ?path)
    (lisp-fun task-tree-node-path ?task ?path))
  
  ;; for dont use this optimization
  ;; (<- (task ?task)
  ;;   (bound ?task)
  ;;   (lisp-pred task-tree-node-p ?task))

  ;; SUBTASK
  (<- (subtask ?task ?subtask)
    (bound ?task)
    (lisp-fun task-children ?task ?children)
    (member ?subtask ?children))

  (<- (subtask ?task ?subtask)
    (not (bound ?task))
    (bound ?subtask)
    (lisp-fun task-tree-node-parent ?subtask ?task)
    (not (== ?task nil)))

  (<- (subtask ?task ?subtask)
    (not (bound ?task))
    (not (bound ?subtask))
    (task ?task)
    (subtask ?task ?subtask))

  ;; SUBTASK+
  (<- (subtask+ ?task ?subtask)
    (subtask ?task ?subtask))

  (<- (subtask+ ?task ?subtask)
    (subtask ?task ?tmp)
    (subtask+ ?tmp ?subtask))

  ;; TASK-SIBLING
  (<- (task-sibling ?task ?sibling)
    (bound ?task)
    (subtask ?parent ?task)
    (subtask ?parent ?sibling)
    (not (== ?sibling ?task)))

  (<- (task-sibling ?task ?sibling)
    (not (bound ?task))
    (subtask ?parent ?sibling)
    (subtask ?parent ?task)
    (not (== ?sibling ?task)))

  ;; TASK-NEXT-SIBLING
  ;; FIXME: we should not simply use temporal, but rater causal relations
  (<- (task-next-sibling ?task ?next)
    (bound ?task)
    (list-of ?sib (task-sibling ?task ?sib) ?siblings)
    (member ?next ?siblings)
    (task-created-at ?task ?ct-task)
    (task-created-at ?next ?ct-next)
    (<= ?ct-task ?ct-next)
    (forall (and
             (member ?other ?siblings)
             (not (== ?next ?other))
             (task-created-at ?other ?ct-other)
             (<= ?ct-task ?ct-other))
            (<= ?ct-next ?ct-other)))
  
  ;; TASK-STATUS-FLUENT
  (<- (task-status-fluent ?task ?fluent)
    (task ?task)
    (lisp-fun task-status-fluent-name ?task ?fluent))

  ;; TASK-GOAL
  (<- (task-goal ?task ?goal)
    (goal-task ?task)
    (lisp-pred goal-task-tree-node-p ?task)
    (lisp-fun goal-task-tree-node-goal ?task ?goal))

  ;; TASK-OUTCOME
  (<- (task-outcome ?task ?outcome)
    (member ?outcome (:succeeded :failed :evaporated))
    (holds (task-status ?task ?outcome) ?_))

  ;; TASK-RESULT
  (<- (task-result ?task ?result)
    (task ?task)
    (lisp-fun task-tree-node-result ?task ?result))

  ;; TASK-ERROR
  (<- (task-error ?task ?error)
    (task-outcome ?task :failed)
    (task-result ?task ?result)
    (lisp-fun extract-task-error ?result ?error))

  ;; ERROR-TYPE
  (<- (error-type ?error ?type)
    (bound ?error)
    (lisp-fun type-of ?error ?type))

  (<- (error-type ?error ?type)
    (not (nound ?error))
    (warn "Trying to call ERROR-TYPE on unbound variable."))

  ;; HOLDS TASK-STATUS
  (<- (holds (task-status ?task ?status) ?t)
    (task-status-fluent ?task ?status-fluent)
    (holds (fluent-value ?status-fluent ?status) ?t))

  ;; TASK-CREATED-AT
  (<- (task-created-at ?task ?time)
    (holds (task-status ?task :created) (at ?time)))

  (<- (task-started-at ?task ?time)
    (task ?task)
    (bagof ?t (holds (task-status ?task :running) (at ?t))
           ?times)
    (sort ?times < (?time . ?_)))

  ;; TASK-ENDED-AT
  (<- (task-ended-at ?task ?time)
    (task ?task)
    (member ?status (:succeeded :failed :evaporated))
    (holds (task-status ?task ?status) (at ?time)))

  ;; TODO(gaya-) this is a very weird place to define such a predicate.
  ;; It's way too specific.
  ;; (<- (task-location-context ?task ?loc)
  ;;   (task ?task)
  ;;   (subtask+ ?loc-task ?task)
  ;;   (task-goal ?loc-task (at-location (?loc)))
  ;;   (task ?loc-task))
  ))
