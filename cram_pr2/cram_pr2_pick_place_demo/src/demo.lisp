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

  (let ((list-of-objects '(;; :breakfast-cereal
                           :milk ;; :cup :bowl :spoon
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

  ;; perform tasks
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
    (task-created-at ?top-level-name ?task ?created-time-task)
    (task-created-at ?top-level-name ?next-task ?created-time-next-task)
    (<= ?created-time-task ?created-time-next-task)
    (forall (and
             (member ?other-next-task ?siblings)
             (task-specific-action ?top-level-name ?subtree-path ?action-type ?other-next-task ?_)
             (not (== ?next-task ?other-next-task))
             (task-created-at ?top-level-name ?other-next-task ?created-time-other-next-task)
             (<= ?created-time-task ?created-time-other-next-task))
            (<= ?created-time-next-task ?created-time-other-next-task)))

  (<- (task-previous-action-sibling ?top-level-name ?subtree-path ?task ?action-type ?next-task)
    (bound ?top-level-name)
    (bound ?subtree-path)
    (bound ?task)
    ;; (bound ?action-type)
    (bagof ?sibling (task-sibling ?task ?sibling) ?siblings)
    (member ?next-task ?siblings)
    (task-specific-action ?top-level-name ?subtree-path ?action-type ?next-task ?_)
    (task-created-at ?top-level-name ?task ?created-time-task)
    (task-created-at ?top-level-name ?next-task ?created-time-next-task)
    (>= ?created-time-task ?created-time-next-task)
    (forall (and
             (member ?other-next-task ?siblings)
             (not (== ?next-task ?other-next-task))
             (task-specific-action ?top-level-name ?subtree-path ?action-type ?other-next-task ?_)
             (task-created-at ?top-level-name ?other-next-task ?created-time-other-next-task)
             (>= ?created-time-task ?created-time-other-next-task))
            (>= ?created-time-next-task ?created-time-other-next-task))))


(defun test ()
  (cet:enable-fluent-tracing)
  (cpl-impl::remove-top-level-task-tree :top-level)

  (pr2-proj:with-simulated-robot
    (demo-random nil))

  (cut:force-ll (prolog:prolog `(task-navigating-action :top-level ((demo-random))
                                                        ?task ?designator))))

(defun test-next-sibling-time ()
  (cet:enable-fluent-tracing)
  (cpl-impl::remove-top-level-task-tree :top-level)

  (pr2-proj:with-simulated-robot
    (demo-random nil))

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
    (demo-random))

  (cut:force-ll
   (prolog:prolog `(and (task-specific-action :top-level ((demo-random)) :fetching ?task ?desig)
                        (task-outcome ?task :failed)
                        (format "desig: ~a~%" ?desig)))))

(defun find-location-for-pick-up-using-occasions ()
  (cet:enable-fluent-tracing)
  (cpl-impl::remove-top-level-task-tree :top-level)

  (proj:with-projection-environment pr2-proj:pr2-bullet-projection-environment
    (cpl-impl::named-top-level (:name :top-level)
      (demo-random nil))

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

(defun find-location-for-pick-up (&optional (projection-run-count 5))
  (flet ((get-pick-up-location (top-level-name path)
           (let* ((bindings
                    (car
                     (prolog:prolog
                      `(and
                        (task-fetching-action ,top-level-name ,path
                                              ?fetching-task ?_)
                        (task-full-path ?fetching-task ?fetching-path)
                        (task-picking-up-action ,top-level-name ?fetching-path
                                                ?picking-up-task ?picking-up-designator)
                        (task-outcome ?picking-up-task :succeeded)
                        (task-previous-action-sibling ,top-level-name ?fetching-path
                                                      ?picking-up-task
                                                      :navigating ?navigating-task)
                        (task-navigating-action ,top-level-name ?fetching-path ?navigating-task
                                                ?navigating-designator)))))
                  (picking-action
                    (cut:var-value '?picking-up-designator bindings))
                  (picking-action-newest
                    (unless (cut:is-var picking-action)
                      (desig:newest-effective-designator picking-action)))
                  (picking-arm
                    (when picking-action-newest
                      (third (desig:reference picking-action-newest))))
                  (navigating-action
                    (cut:var-value '?navigating-designator bindings))
                  (navigating-action-newest
                    (unless (cut:is-var navigating-action)
                      (desig:newest-effective-designator navigating-action)))
                  (picking-location
                    (when navigating-action
                      (desig:newest-effective-designator
                       (desig:desig-prop-value navigating-action :location)))))
             (list picking-location picking-arm))))

    (cet:enable-fluent-tracing)
    (cpl-impl::remove-top-level-task-tree :top-level)

    (let (paths)
      (proj:with-projection-environment pr2-proj:pr2-bullet-projection-environment
        (cpl-impl::named-top-level (:name :top-level)
          (dotimes (n projection-run-count)
            (push (demo-random nil) paths))))

      (mapcar (alexandria:curry #'get-pick-up-location :top-level)
              paths))))

(defun find-location-for-pick-up-with-successful-put-down (&optional (projection-run-count 5))
  (flet ((get-pick-up-location (top-level-name path)
           (let* ((bindings
                    (car
                     (prolog:prolog
                      `(and
                        (task-fetching-action ,top-level-name ,path
                                              ?fetching-task ?_)
                        (task-full-path ?fetching-task ?fetching-path)
                        (task-picking-up-action ,top-level-name ?fetching-path
                                                ?picking-up-task ?picking-up-designator)
                        (task-outcome ?picking-up-task :succeeded)
                        (task-previous-action-sibling ,top-level-name ?fetching-path
                                                      ?picking-up-task
                                                      :navigating ?navigating-task)
                        (task-navigating-action ,top-level-name ?fetching-path ?navigating-task
                                                ?navigating-designator)
                        (task-delivering-action ,top-level-name ,path
                                                ?delivering-task ?_)
                        (task-outcome ?delivering-task :succeeded)))))
                  (picking-action
                    (cut:var-value '?picking-up-designator bindings))
                  (picking-action-newest
                    (unless (cut:is-var picking-action)
                      (desig:newest-effective-designator picking-action)))
                  (picking-arm
                    (when picking-action-newest
                      (third (desig:reference picking-action-newest))))
                  (navigating-action
                    (cut:var-value '?navigating-designator bindings))
                  (navigating-action-newest
                    (unless (cut:is-var navigating-action)
                      (desig:newest-effective-designator navigating-action)))
                  (picking-location
                    (when navigating-action-newest
                      (desig:newest-effective-designator
                       (desig:desig-prop-value navigating-action-newest :location)))))
             (list picking-location picking-arm))))

    (cet:enable-fluent-tracing)
    (cpl-impl::remove-top-level-task-tree :top-level)

    (let (paths)
      (proj:with-projection-environment pr2-proj:pr2-bullet-projection-environment
        (cpl-impl::named-top-level (:name :top-level)
          (dotimes (n projection-run-count)
            (push (demo-random nil) paths))))

      (mapcar (alexandria:curry #'get-pick-up-location :top-level)
              paths))))

