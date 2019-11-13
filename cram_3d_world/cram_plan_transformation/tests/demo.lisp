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

(defun init-projection ()
  (def-fact-group costmap-metadata (costmap:costmap-size
                                    costmap:costmap-origin
                                    costmap:costmap-resolution
                                    costmap:orientation-samples
                                    costmap:orientation-sample-step)
    (<- (costmap:costmap-size 12 12))
    (<- (costmap:costmap-origin -6 -6))
    (<- (costmap:costmap-resolution 0.04))
    (<- (costmap:orientation-samples 2))
    (<- (costmap:orientation-sample-step 0.3)))

  (setf cram-bullet-reasoning-belief-state:*robot-parameter* "robot_description")
  (setf cram-bullet-reasoning-belief-state:*kitchen-parameter* "kitchen_description")

  (cram-occasions-events:clear-belief)

  (setf cram-tf:*tf-default-timeout* 2.0)

  (setf prolog:*break-on-lisp-errors* t)

  (cram-bullet-reasoning:clear-costmap-vis-object)

  (setf cram-tf:*tf-broadcasting-enabled* t)

  (setf proj-reasoning::*projection-reasoning-enabled* nil)

  (btr:add-objects-to-mesh-list "cram_bullet_reasoning"))

(roslisp-utilities:register-ros-init-function init-projection)

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

(cpl:def-cram-function both-hands-demo (&optional (list-of-objects '(:bowl :cup)))
  "Transports a bowl and a cup from the sink area to the kitchen island."
  (roslisp:ros-info (plt-tests) "BOTH-HANDS-DEMO start.")
  (initialize)
  (spawn-objects-on-sink-counter)
  (park-robot)
  (let ((object-fetching-locations
          `((:breakfast-cereal . ,(desig:a location
                                           (on (desig:an object
                                                         (type counter-top)
                                                         (urdf-name sink-area-surface)
                                                         (owl-name "kitchen_sink_block_counter_top")
                                                         (part-of kitchen)))
                                           (side left)
                                           (side front)
                                           (range 0.5)))
            (:cup . ,(desig:a location
                              (side left)
                              (on (desig:an object
                                            (type counter-top)
                                            (urdf-name sink-area-surface)
                                            (owl-name "kitchen_sink_block_counter_top")
                                            (part-of kitchen)))))
            (:bowl . ,(desig:a location
                               (on (desig:an object
                                             (type counter-top)
                                             (urdf-name sink-area-surface)
                                             (owl-name "kitchen_sink_block_counter_top")
                                             (part-of kitchen)))
                               (side left)
                               (side front)
                               (range-invert 0.5)))))
        (object-placing-locations
          (let ((?pose
                  (cl-transforms-stamped:make-pose-stamped
                   "map"
                   0.0
                   (cl-transforms:make-3d-vector -0.78 0.8 0.95)
                   (cl-transforms:make-quaternion 0 0 0.6 0.4))))
            `((:breakfast-cereal . ,(desig:a location
                                             (pose ?pose)))
              (:cup . ,(desig:a location
                                (right-of (an object (type bowl)))
                                (behind (an object (type bowl)))
                                (near (an object (type bowl)))
                                (for (an object (type cup)))))
              (:bowl . ,(desig:a location
                                 (on (desig:an object
                                               (type counter-top)
                                               (urdf-name kitchen-island-surface)
                                               (owl-name "kitchen_island_counter_top")
                                               (part-of kitchen)))
                                 (context table-setting)
                                 (for (an object (type bowl)))
                                 (object-count 3)
                                 (side back)
                                 (side right)
                                 (range-invert 0.5)))))))

    (dolist (?object-type list-of-objects)
      (let* ((?fetching-location
               (cdr (assoc ?object-type object-fetching-locations)))
             (?delivering-location
               (cdr (assoc ?object-type object-placing-locations)))
             (?object-to-fetch
               (desig:an object
                         (type ?object-type)
                         (location ?fetching-location))))

        (when (eq ?object-type :bowl)
          (cpl:with-failure-handling
              ((common-fail:high-level-failure (e)
                 (roslisp:ros-warn (pp-plans demo) "Failure happened: ~a~%Skipping the search" e)
                 (return)))
            (let ((?loc (cdr (assoc :breakfast-cereal object-fetching-locations))))
              (exe:perform
               (desig:an action
                         (type searching)
                         (object (desig:an object (type breakfast-cereal)))
                         (location ?loc))))))

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

  (park-robot)
  (sb-ext:gc :full t)
  (roslisp:ros-info (plt-tests) "BOTH-HANDS-DEMO finished."))

(cpl:def-cram-function environment-demo (&optional (list-of-objects '(:spoon :fork)))
  "Transports a spoon and a fork from the kitchen drawer to the kitchen island."
  (roslisp:ros-info (plt-tests) "ENVIRONMENT-DEMO start.")
  (initialize)
  (spawn-objects-on-sink-counter)
  (park-robot)

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
  (park-robot)
  (sb-ext:gc :full t)
  (roslisp:ros-info (plt-tests) "ENVIRONMENT-DEMO finished.")
  cpl:*current-path*)




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

(defun initialize ()
  (sb-ext:gc :full t)
  (setf proj-reasoning::*projection-checks-enabled* t)
  (btr:detach-all-objects (btr:get-robot-object))
  (btr:detach-all-objects (btr:get-environment-object))
  (btr-utils:kill-all-objects)
  (setf (btr:joint-state (btr:get-environment-object)
                         "sink_area_left_upper_drawer_main_joint")
        0.0)
  (btr-belief::publish-environment-joint-state
   (btr:joint-states (btr:get-environment-object)))

  (setf desig::*designators* (tg:make-weak-hash-table :weakness :key))

  (coe:clear-belief))

(cpl:def-cram-function park-robot ()
  (cpl:with-failure-handling
      ((cpl:plan-failure (e)
         (declare (ignore e))
         (return)))
    (cpl:par
      (exe:perform
       (desig:an action
                 (type positioning-arm)
                 (left-configuration park)
                 (right-configuration park)))
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
      (exe:perform (desig:an action (type opening-gripper) (gripper (left right))))
      (exe:perform (desig:an action (type looking) (direction forward))))))

(defparameter *object-spawning-poses*
  '((:breakfast-cereal . ((1.4 0.4 0.85) (0 0 0 1)))
    (:cup . ((1.3 0.6 0.9) (0 0 0 1)))
    (:bowl . ((1.4 0.8 0.87) (0 0 0 1)))
    (:spoon . ((1.43 0.9 0.74132) (0 0 0 1)))
    (:fork . ((1.45 1.05 0.74132) (0 0 0 1)))))

(defun spawn-objects-on-sink-counter (&optional (spawning-poses *object-spawning-poses*))
  (btr-utils:kill-all-objects)
  (btr:add-objects-to-mesh-list "cram_bullet_reasoning")
  (btr:detach-all-objects (btr:get-robot-object))
  (let ((object-types '(:breakfast-cereal :cup :bowl :spoon :fork)))
    ;; spawn objects at default poses
    (let ((objects (mapcar (lambda (object-type)
                             (let* ((object-name
                                      (intern (format nil "~a-1" object-type) :keyword))
                                    (object-pose-list
                                      (cdr (assoc object-type spawning-poses)))
                                    (object
                                      (btr-utils:spawn-object
                                       object-name
                                       object-type
                                       :pose object-pose-list)))
                               (btr-utils:move-object
                                (btr:name object)
                                (cram-tf:rotate-pose
                                 (btr:pose object) :z (/ (* 2 pi) (random 10.0))))
                               object))
                           object-types)))
      ;; stabilize world
      (btr:simulate btr:*current-bullet-world* 100)
      objects))

  (btr:attach-object (btr:object btr:*current-bullet-world* :kitchen)
                     (btr:object btr:*current-bullet-world* :spoon-1)
                     :link "sink_area_left_upper_drawer_main")
  (btr:attach-object (btr:object btr:*current-bullet-world* :kitchen)
                     (btr:object btr:*current-bullet-world* :fork-1)
                     :link "sink_area_left_upper_drawer_main"))
