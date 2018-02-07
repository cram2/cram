;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors
;;;       may be used to endorse or promote products derived from this software
;;;       without specific prior written permission.
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

(in-package :demo)

(defparameter *object-cad-models*
  '((:cup . "cup_eco_orange")
    (:bowl . "edeka_red_bowl")))

(defmacro with-real-robot (&body body)
  `(cram-process-modules:with-process-modules-running
       (rs:robosherlock-perception-pm
        pr2-pms::pr2-base-pm pr2-pms::pr2-arms-pm
        pr2-pms::pr2-grippers-pm pr2-pms::pr2-ptu-pm)
     (cpl:top-level
       ,@body)))

(cpl:def-cram-function initialize-or-finalize ()
  (cpl:with-failure-handling
      ((cpl:plan-failure (e)
         (declare (ignore e))
         (return)))
    (cpl:par
      (pp-plans::park-arms)
      (let ((?pose (cl-transforms-stamped:make-pose-stamped
                    cram-tf:*fixed-frame*
                    0.0
                    (cl-transforms:make-identity-vector)
                    (cl-transforms:make-identity-rotation))))
        (exe:perform
         (desig:an action
                   (type going)
                   (target (desig:a location
                                    (pose ?pose))))))
      (exe:perform (desig:an action (type opening) (gripper (left right))))
      (exe:perform (desig:an action (type looking) (direction forward))))))

(cpl:def-cram-function demo-random (&optional (random t))
  (btr:detach-all-objects (btr:get-robot-object))
  (btr-utils:kill-all-objects)

  (when (eql cram-projection:*projection-environment*
             'cram-pr2-projection::pr2-bullet-projection-environment)
    (if random
        (spawn-objects-on-sink-counter-randomly)
        (spawn-objects-on-sink-counter)))

  (setf cram-robot-pose-guassian-costmap::*orientation-samples* 1)

  (initialize-or-finalize)

  (let ((list-of-objects '(:breakfast-cereal; :milk :cup :bowl :spoon
                           )))
    (dolist (?object-type list-of-objects)
      (let* ((?cad-model
               (cdr (assoc ?object-type *object-cad-models*)))
             (?object-to-fetch
               (desig:an object
                         (type ?object-type)
                         (desig:when ?cad-model
                           (cad-model ?cad-model))))
             (?fetching-location
               (desig:a location
                        (on "CounterTop")
                        (name "iai_kitchen_sink_area_counter_top")
                        (side left)))
             (?placing-target-pose
               (cl-transforms-stamped:pose->pose-stamped
                "map" 0.0
                (cram-bullet-reasoning:ensure-pose
                 (cdr (assoc ?object-type *object-placing-poses*)))))
             (?arm-to-use
               (cdr (assoc ?object-type *object-grasping-arms*)))
             (?delivering-location
               (desig:a location
                        (pose ?placing-target-pose))))

        (cpl:with-failure-handling
            ((common-fail:high-level-failure (e)
               (roslisp:ros-warn (pp-plans demo) "Failure happened: ~a~%Skipping..." e)
               (return)))
          (exe:perform
           (desig:an action
                     (type transporting)
                     (object ?object-to-fetch)
                     ;; (arm ?arm-to-use)
                     (location ?fetching-location)
                     (target ?delivering-location)))))))

  (initialize-or-finalize)

  cpl:*current-path*)



(defgeneric extract-task-error (err)
  (:method ((err cpl:plan-failure))
    err)
  (:method ((err cpl:common-lisp-error-envelope))
    (cpl:envelop-error err)))

(def-fact-group tasks ()

  ;; top-level
  (<- (top-level-task ?top-level-name ?top-level-task-node)
    (bound ?top-level-name)
    (lisp-fun cpl:get-top-level-task-tree ?top-level-name ?top-level-task-node))

  ;; util
  (<- (task-full-path ?task-node ?path)
    (bound ?task-node)
    (lisp-fun cpl:task-tree-node-path ?task-node ?path))

  ;; tasks of top-level
  (<- (task-of-top-level ?top-level-name ?task-node)
    (bound ?top-level-name)
    (top-level-task ?top-level-name ?top-level-task-node)
    (lisp-fun cpl:flatten-task-tree ?top-level-task-node ?all-task-nodes)
    (member ?task-node ?all-task-nodes))

  ;; task for subtree
  (<- (task ?top-level-name ?subtree-path ?task-node)
    (bound ?top-level-name)
    (bound ?subtree-path)
    (top-level-task ?top-level-name ?top-level-task)
    (lisp-fun cpl:task-tree-node ?subtree-path ?top-level-task ?subtree-task)
    (lisp-fun cpl:flatten-task-tree ?subtree-task ?all-subtree-tasks)
    (member ?task-node ?all-subtree-tasks))

  ;; subtask
  (<- (subtask ?task ?subtask)
    (bound ?task)
    (lisp-fun cpl:task-tree-node-children ?task ?children)
    (member (?_ . ?subtask) ?children))

  (<- (subtask ?task ?subtask)
    (not (bound ?task))
    (bound ?subtask)
    (lisp-fun task-tree-node-parent ?subtask ?task)
    (not (== ?task nil)))

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
    (lisp-fun cpl:value ?fluent ?value))

  (<- (task-status ?task ?status)
    (bound ?task)
    (task-status-fluent ?task ?fluent)
    (fluent-value ?fluent ?status))

  ;; (<- (holds (fluent-value ?fluent ?value) ?t)
  ;;   (bound ?fluent)
  ;;   (lisp-fun cet:episode-knowledge-fluent-durations ?fluent ?durations)
  ;;   (member (?value . ?duration) ?durations)
  ;;   (duration-includes ?duration ?t))

  ;; ;; HOLDS TASK-STATUS
  ;; (<- (holds (task-status ?task ?status) ?t)
  ;;   (task-status-fluent ?task ?status-fluent)
  ;;   (holds (cpl:fluent-value ?status-fluent ?status) ?t))

  ;; task-outcome
  (<- (task-outcome ?task ?outcome)
    (bound ?task)
    (member ?outcome (:succeeded :failed :evaporated))
    ;; (holds (task-status ?task ?outcome) ?_)
    (task-status ?task ?outcome))

  ;; task-error
  (<- (task-error ?task ?error)
    (bound ?task)
    (task-outcome ?task :failed)
    (task-result ?task ?result)
    (lisp-fun extract-task-error ?result ?error))

  ;; ;; TASK-CREATED-AT
  ;; (<- (task-created-at ?task ?time)
  ;;   (holds (task-status ?task :created) (at ?time)))

  ;; (<- (task-started-at ?task ?time)
  ;;   (task ?task)
  ;;   (bagof ?t (holds (task-status ?task :running) (at ?t))
  ;;          ?times)
  ;;   (sort ?times < (?time . ?_)))

  ;; ;; TASK-ENDED-AT
  ;; (<- (task-ended-at ?task ?time)
  ;;   (task ?task)
  ;;   (member ?status (:succeeded :failed :evaporated))
  ;;   (holds (task-status ?task ?status) (at ?time)))

  ;; perform TASK-GOAL
  (<- (perform-task-of-top-level ?top-level-name ?task-node)
    (bound ?top-level-name)
    (task-of-top-level ?top-level-name ?task-node)
    (lisp-fun cpl:task-tree-node-path ?task-node (?path . ?_))
    (equal ?path (cpl:goal (perform ?_) . ?_)))

  (<- (perform-task ?top-level-name ?subtree-path ?task-node)
    (bound ?top-level-name)
    (bound ?subtree-path)
    (task ?top-level-name ?subtree-path ?task-node)
    (lisp-fun cpl:task-tree-node-path ?task-node (?path . ?_))
    (equal ?path (cpl:goal (perform ?_) . ?_)))

  ;; TASK-NEXT-SIBLING
  ;; FIXME: we should not simply use temporal, but rater causal relations
  ;; (<- (task-next-sibling ?task ?next)
  ;;   (bound ?task)
  ;;   (bagof ?sib (task-sibling ?task ?sib) ?siblings)
  ;;   (member ?next ?siblings)
  ;;   (task-created-at ?task ?ct-task)
  ;;   (task-created-at ?next ?ct-next)
  ;;   (<= ?ct-task ?ct-next)
  ;;   (forall (and
  ;;            (member ?other ?siblings)
  ;;            (not (== ?next ?other))
  ;;            (task-created-at ?other ?ct-other)
  ;;            (<= ?ct-task ?ct-other))
  ;;           (<= ?ct-next ?ct-other)))

  (<- (task-specific-action ?top-level-name ?subtree-path ?action-type ?param)
    (bound ?top-level-name)
    (bound ?subtree-path)
    (perform-task ?top-level-name ?subtree-path ?task)
    (task-parameter ?task ?param)
    (lisp-type ?param desig:action-designator)
    (spec:property ?param (:type ?action-type)))

  (<- (task-navigating-action ?top-level-name ?subtree-path ?designator)
    (task-specific-action ?top-level-name ?subtree-path :navigating ?designator))

  (<- (task-fetching-action ?top-level-name ?subtree-path ?designator)
    (task-specific-action ?top-level-name ?subtree-path :fetching ?designator)))


(defun test ()
  (pr2-proj:with-simulated-robot
    (let ((path-1 (demo-random nil))
          (path-2 (demo-random nil)))
      (append
       (cut:force-ll (prolog:prolog `(task-navigating-action :top-level ,path-1 ?designator)))
       (cut:force-ll (prolog:prolog `(task-navigating-action :top-level ,path-2 ?designator)))))))
