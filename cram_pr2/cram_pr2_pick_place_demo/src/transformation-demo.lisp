;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;               2019, Arthur Niedzwiecki <aniedz@cs.uni-bremen.de>
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

(defun demo-plan-transformation (&optional (reset-task-tree T) (rule-function 'demo-both-hands))
  (unless (member rule-function '(demo-both-hands demo-environment))
    (error "Demo function unknown. Choose ~a or ~a"
           'demo-both-hands
           'demo-environment))
  (unless (eq (roslisp:node-status) :RUNNING)
    (roslisp-utilities:startup-ros))
  (cet:enable-fluent-tracing)
  (when reset-task-tree
    (cpl-impl::remove-top-level-task-tree (plt:get-top-level-name)))
  (funcall rule-function)
  (cet:disable-fluent-tracing)
  (plt:apply-rules)
  (funcall rule-function))

(defun demo-both-hands ()
  (urdf-proj:with-projected-robot
      (demo-random nil '(:bowl :breakfast-cereal))))

(defun demo-environment ()
  (urdf-proj:with-projected-robot
      (demo-environment-rule)))

(cpl:def-cram-function demo-environment-rule (&optional
                                              (list-of-objects '(:spoon :fork)))
  (initialize)
  (spawn-objects-on-sink-counter)
  (btr-utils:spawn-object :fork-1
                          :fork
                          :pose '((1.45 1.05 0.74132) (0 0 0 1)))
  (btr:attach-object (btr:object btr:*current-bullet-world* :kitchen)
                     (btr:object btr:*current-bullet-world* :fork-1)
                     :link "sink_area_left_upper_drawer_main")
  (park-robot)

  (let ((object-fetching-locations
          `((:spoon . ,(desig:a location
                                (in (desig:an object
                                              (type drawer)
                                              (urdf-name sink-area-left-upper-drawer-main)
                                              (owl-name "drawer_sinkblock_upper_open")
                                              (part-of kitchen)))
                                (side front)))
            (:fork . ,(desig:a location
                                (in (desig:an object
                                              (type drawer)
                                              (urdf-name sink-area-left-upper-drawer-main)
                                              (owl-name "drawer_sinkblock_upper_open")
                                              (part-of kitchen)))
                                (side front)))))
        (object-placing-locations
          (let ((?spoon-pose (cl-transforms-stamped:make-pose-stamped
                              "map"
                              0.0
                              (cl-transforms:make-3d-vector -0.78 1.5 0.86)
                              (cl-transforms:make-quaternion 0 0 0 1)))
                (?fork-pose (cl-transforms-stamped:make-pose-stamped
                             "map"
                             0.0
                             (cl-transforms:make-3d-vector -0.78 1.6 0.86)
                             (cl-transforms:make-quaternion 0 0 0 1))))
            `((:fork . ,(desig:a location
                                 (pose ?fork-pose)))
              (:spoon . ,(desig:a location
                                  (pose ?spoon-pose)))))))

    (dolist (?object-type list-of-objects)
      (let* ((?fetching-location
               (cdr (assoc ?object-type object-fetching-locations)))
             (?delivering-location
               (cdr (assoc ?object-type object-placing-locations)))
             (?arm-to-use
               (cdr (assoc ?object-type *object-grasping-arms*)))
             (?cad-model
               (cdr (assoc ?object-type *object-cad-models*)))
             (?color
               (cdr (assoc ?object-type *object-colors*)))
             (?object-to-fetch
               (desig:an object
                         (type ?object-type)
                         (location ?fetching-location)
                         (desig:when ?cad-model
                           (cad-model ?cad-model))
                         (desig:when ?color
                           (color ?color)))))
        (cpl:with-failure-handling
            ((common-fail:high-level-failure (e)
               (roslisp:ros-warn (pp-plans demo) "Failure happened: ~a~%Skipping..." e)
               (return)))
          (exe:perform
           (desig:an action
                     (type transporting)
                     (object ?object-to-fetch)
                     (desig:when ?arm-to-use
                       (arm ?arm-to-use))
                     (location ?fetching-location)
                     (target ?delivering-location)))))))
  (park-robot)
  (finalize)
  cpl:*current-path*)
