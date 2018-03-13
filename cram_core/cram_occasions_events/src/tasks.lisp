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

;; Note: #demmeln: There is much potential for optimization, e.g. chaching the
;; task and fluent lists, only checking for correct type if task or fluents
;; are already bound etc...

(def-fact-group fluents (holds)
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
  )
