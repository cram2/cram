;;;
;;; Copyright (c) 2019, Arthur Niedzwiecki <aniedz@cs.uni-bremen.de>
;;;                     
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

(in-package :plt-tests)

(defun move-to-origin ()
  "Navigating the robot will reset the rigid bodies of the objects,
such that they can fall down and collide with the supporting surface.
This is a workaround until PR #120 is merged."
  (let* ((?pose
           (cl-transforms-stamped:make-pose-stamped 
            "map" 0.0
            (cl-transforms:make-3d-vector 0 0 0)
            (cl-tf:make-identity-rotation)))
         (?target-robot-location (desig:a location
                                          (pose ?pose))))
    (exe:perform (desig:an action
                           (type navigating)
                           (location ?target-robot-location)))))

(defun demo-successful (&rest objects)
  "Check if all `objects' are on the kitchen table." 
  (btr:simulate btr:*current-bullet-world* 100)
  (prolog `(and (btr:bullet-world ?w)
                (forall (member ?obj ,objects)
                        (btr:contact ?w ?obj :kitchen "kitchen_island")))))

(defun execute-demo (&optional demo-function objects-manipulated (reset-tt nil) (retries 5))
  "Executes the `demo-function' until the `objects-manipulated' are at the right spot,
but only up to `retries' times. If `reset-tt' is T the task tree is wiped clean,
if not, the demo is executed with all changes made in the task tree (e.g. plan transformations)."
  (when (>= retries 0)
    (when reset-tt
      (plt:reset-task-tree))
    (urdf-proj:with-projected-robot
      (cet:enable-fluent-tracing)
      (funcall demo-function)
      (cet:disable-fluent-tracing)
      (move-to-origin))
    (or (apply #'demo-successful objects-manipulated)
        (execute-demo demo-function objects-manipulated reset-tt (decf retries)))))

(defun both-hands-demo ()
  "Wrapper for the pp-demo, transporting a bowl and a cup from the sink area to the kitchen island."
  (roslisp:ros-info (plt-tests) "BOTH-HANDS-DEMO start.")
  (demo::demo-random nil '(:bowl :cup))
  (roslisp:ros-info (plt-tests) "BOTH-HANDS-DEMO finished."))

(cpl:def-cram-function environment-demo (&optional (list-of-objects '(:spoon :fork)))
  "Transports a spoon and a fork from the kitchen drawer to the kitchen island."
  (roslisp:ros-info (plt-tests) "ENVIRONMENT-DEMO start.")
  (demo::initialize)
  (demo::spawn-objects-on-sink-counter)
  (btr-utils:spawn-object :fork-1 :fork :pose '((1.45 1.05 0.74132) (0 0 0 1)))
  (btr:attach-object (btr:object btr:*current-bullet-world* :kitchen)
                     (btr:object btr:*current-bullet-world* :fork-1)
                     :link "sink_area_left_upper_drawer_main")
  (demo::park-robot)

  (let ((object-fetching-locations
          `((:spoon . ,(desig:a
                        location
                        (in (desig:an object
                                      (type drawer)
                                      (urdf-name sink-area-left-upper-drawer-main)
                                      (owl-name "drawer_sinkblock_upper_open")
                                      (part-of kitchen)))
                        (side front)))
            (:fork . ,(desig:a
                       location
                       (in (desig:an object
                                     (type drawer)
                                     (urdf-name sink-area-left-upper-drawer-main)
                                     (owl-name "drawer_sinkblock_upper_open")
                                     (part-of kitchen)))
                                (side front)))))
        (object-placing-locations
          (let ((?spoon-pose (cl-transforms-stamped:make-pose-stamped
                              "map" 0.0
                              (cl-transforms:make-3d-vector -0.78 1.5 0.86)
                              (cl-transforms:make-quaternion 0 0 0 1)))
                (?fork-pose (cl-transforms-stamped:make-pose-stamped
                             "map" 0.0
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
             (?object-to-fetch
               (desig:an object
                         (type ?object-type)
                         (location ?fetching-location))))
        (cpl:with-failure-handling
            ((common-fail:high-level-failure (e)
               (roslisp:ros-warn (pp-plans demo) "Failure happened: ~a~%Skipping..." e)
               (return)))
          (exe:perform
           (desig:an action
                     (type transporting)
                     (object ?object-to-fetch)
                     (location ?fetching-location)
                     (target ?delivering-location)))))))
  (demo::park-robot)
  (demo::finalize)
  (roslisp:ros-info (plt-tests) "ENVIRONMENT-DEMO finished.")
  cpl:*current-path*)
