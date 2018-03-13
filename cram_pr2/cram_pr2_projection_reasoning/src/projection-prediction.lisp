;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :pr2-proj-reasoning)

(defparameter *projection-reasoning-enabled* t)

(defun extract-successful-transporting-designators (top-level-name path)
  (let* ((bindings
           (car
            (prolog:prolog
             `(and
               ;; find successful transporting action
               ;; (task-specific-action ,top-level-name ,path :transporting
               ;;                       ?transporting-task ?_)
               ;; (task-outcome ?transporting-task :succeeded)
               ;; (task-full-path ?transporting-task ?transporting-path)
               (task-specific-action ,top-level-name ;; ?transporting-path
                                     ,path
                                     :fetching
                                     ?fetching-task ?_)
               (task-full-path ?fetching-task ?fetching-path)
               (task-specific-action ,top-level-name ?fetching-path :picking-up
                                     ?picking-up-task ?picking-up-designator)
               (task-outcome ?picking-up-task :succeeded)
               ;; make sure that the corresponding delivering action succeeded
               (task-specific-action ,top-level-name ;; ?transporting-path
                                     ,path
                                     :delivering
                                     ?delivering-task ?_)
               (task-outcome ?delivering-task :succeeded)

               ;; find closest navigation action before pick-up
               (task-previous-action-sibling ,top-level-name ?fetching-path
                                             ?picking-up-task
                                             :navigating ?picking-navigating-task)
               (task-specific-action ,top-level-name ?fetching-path :navigating
                                     ?picking-navigating-task
                                     ?picking-navigating-designator)
               ;; find closest navigation action before place
               (task-full-path ?delivering-task ?delivering-path)
               (task-specific-action ,top-level-name ?delivering-path :placing
                                     ?placing-task ?placing-designator)
               (task-outcome ?placing-task :succeeded)
               (task-previous-action-sibling ,top-level-name ?delivering-path
                                             ?placing-task
                                             :navigating ?placing-navigating-task)
               (task-specific-action ,top-level-name ?delivering-path :navigating
                                     ?placing-navigating-task
                                     ?placing-navigating-designator)
               ;; ;; calculate navigation distances
               ;; (btr:timeline ?timeline)
               ;; (bagof ?distance
               ;;        (and
               ;;         (task-specific-action ,top-level-name ?transporting-task
               ;;                               :navigating ?navigating-task ?_)
               ;;         (task-started-at ,top-level-name ?navigating-task ?start-time)
               ;;         (coe:holds ?timeline (cpoe:loc ?robot ?start-location)
               ;;                    (coe:at ?start-time))
               ;;         (task-ended-at ,top-level-name ?navigating-task ?end-time)
               ;;         (coe:holds ?timeline (cpoe:loc ?robot ?end-location)
               ;;                    (coe:at ?end-time))))
               ))))
         (picking-action
           (cut:var-value '?picking-up-designator bindings))
         (picking-action-newest
           (unless (cut:is-var picking-action)
             (desig:newest-effective-designator picking-action)))

         (picking-navigating-action
           (cut:var-value '?picking-navigating-designator bindings))
         (picking-navigating-action-newest
           (unless (cut:is-var picking-navigating-action)
             (desig:newest-effective-designator picking-navigating-action)))
         (picking-location-newest
           (when picking-navigating-action-newest
             (desig:newest-effective-designator
              (desig:desig-prop-value picking-navigating-action-newest :location))))

         (placing-action
           (cut:var-value '?placing-designator bindings))
         (placing-action-newest
           (unless (cut:is-var placing-action)
             (desig:newest-effective-designator placing-action)))

         (placing-navigating-action
           (cut:var-value '?placing-navigating-designator bindings))
         (placing-navigating-action-newest
           (unless (cut:is-var placing-navigating-action)
             (desig:newest-effective-designator placing-navigating-action)))
         (placing-location-newest
           (when placing-navigating-action-newest
             (desig:newest-effective-designator
              (desig:desig-prop-value placing-navigating-action-newest :location)))))
    (list picking-location-newest picking-action-newest
          placing-location-newest placing-action-newest)))

(defun estimate-distance-between-pose-stamped (pose-stamped-1 pose-stamped-2
                                               &optional
                                                 (translational-weight 1.0)
                                                 (rotational-weight 1.0))
  (assert pose-stamped-1)
  (assert pose-stamped-2)
  (assert (string-equal (cl-transforms-stamped:frame-id pose-stamped-1)
                        (cl-transforms-stamped:frame-id pose-stamped-2)))
  (+ (* translational-weight
        (cl-transforms:v-dist
         (cl-transforms:origin pose-stamped-1)
         (cl-transforms:origin pose-stamped-2)))
     (* rotational-weight
        (abs
         (cl-transforms:normalize-angle
          (cl-transforms:angle-between-quaternions
           (cl-transforms:orientation pose-stamped-1)
           (cl-transforms:orientation pose-stamped-2)))))))

(defun estimate-distances-starting-from-current-pose (current-pose-stamped list-of-pose-stampeds)
  (unless (some #'null list-of-pose-stampeds)
    (reduce #'+
            (maplist (lambda (sublist)
                       (if (>= (length sublist) 2)
                           (estimate-distance-between-pose-stamped
                            (first sublist)
                            (second sublist))
                           0.0))
                     (cons current-pose-stamped list-of-pose-stampeds)))))

(defun calculate-index-of-shortest-route (current-pose-stamped lists-of-pose-stampeds)
  (let* ((distances
           (mapcar (alexandria:curry #'estimate-distances-starting-from-current-pose
                                     current-pose-stamped)
                   lists-of-pose-stampeds))
         (min-distances
           (reduce (lambda (x y)
                     (if x
                         (if y
                             (min x y)
                             x)
                         (if y
                             y
                             NIL)))
                   distances)))
    (values (position min-distances distances)
            min-distances)))

(defun pick-best-parameters-by-distance (paths)
  (let* ((parameter-lists
           (mapcar (alexandria:curry #'extract-successful-transporting-designators :top-level)
                   paths))
         (parameter-lists-only-poses
           (mapcar (lambda (set-of-params)
                     (list (when (first set-of-params)
                             (desig:reference (first set-of-params)))
                           (when (third set-of-params)
                             (desig:reference (third set-of-params)))))
                   parameter-lists))
         (current-robot-pose
           ;; todo: actually should use cram-tf:robot-current-pose
           (cl-transforms-stamped:pose->pose-stamped
            cram-tf:*fixed-frame*
            0.0
            (btr:pose (btr:get-robot-object))))
         (best-parameter-list-index
           (calculate-index-of-shortest-route
            current-robot-pose
            parameter-lists-only-poses))
         (best-parameters
           (nth best-parameter-list-index parameter-lists)))
    best-parameters))

(defmacro with-projected-task-tree (designators number-of-runs cost-function &body body)
  (alexandria:with-gensyms (paths)
    `(if (or cram-projection:*projection-environment*
             (not *projection-reasoning-enabled*))
         (progn
           ,@body)
         (let* (,paths
                (world btr:*current-bullet-world*)
                (world-state (btr::get-state world)))
           (time
            (unwind-protect
                 (proj:with-projection-environment pr2-proj:pr2-bullet-projection-environment
                   (cpl:with-tags
                     ,@(loop for i to (1- number-of-runs)
                             collecting
                             (let ((task-variable (gensym "PREDICTION-TASK-")))
                               `(progn
                                  (:tag ,task-variable
                                    (btr::restore-world-state world-state world)
                                    (pr2-proj::set-tf-from-bullet)
                                    ,@body)
                                  (push (cpl:task-path ,task-variable) ,paths))))))
              (btr::restore-world-state world-state world)))
           (destructuring-bind ,designators
               (time (funcall ,cost-function ,paths))
             ,@body)))))
