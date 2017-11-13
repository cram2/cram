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

(defparameter *sink-nav-goal*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector 0.75d0 0.70d0 0.0)
   (cl-transforms:make-identity-rotation)))
(defparameter *island-nav-goal*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -0.2d0 1.5d0 0.0)
   (cl-transforms:make-quaternion 0 0 1 0)))
(defparameter *look-goal*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint"
   0.0
   (cl-transforms:make-3d-vector 0.5d0 0.0d0 1.0d0)
   (cl-transforms:make-identity-rotation)))

(defparameter *object-spawning-poses*
  '((:cereal . ((1.4 0.4 0.95) (0 0 0 1)))
    (:cup . ((1.3 0.6 0.9) (0 0 0 1)))
    (:bowl . ((1.4 0.8 0.87) (0 0 0 1)))
    (:spoon . ((1.4 1.0 0.86) (0 0 0 1)))
    (:milk . ((1.5 0.6 0.95) (0 0 0 1)))))
(defparameter *object-placing-poses*
  '((:cereal . ((-0.85 1.7 0.95) (0 0 0 1)))
    (:cup . ((-0.9 1.4 0.9) (0 0 0.7071 0.7071)))
    (:bowl . ((-0.75 1.3 0.89) (0 0 0 1)))
    (:spoon . ((-0.78 1.45 0.86) (0 0 1 0)))
    (:milk . ((-0.79 1.0 0.95) (0 0 0.7071 0.7071)))))

(defparameter *object-grasping-arms*
  '((:cereal . :right)
    (:cup . :right)
    (:bowl . :right)
    (:spoon . :left)
    (:milk . :right)))

(defmacro with-simulated-robot (&body body)
  `(let ((results
           (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
             (cpl:top-level
               ,@body))))
     (car (cram-projection::projection-environment-result-result results))))

;; (defmacro with-real-robot (&body body)
;;   `(cram-process-modules:with-process-modules-running
;;        (pr2-pms::pr2-perception-pm pr2-pms::pr2-base-pm pr2-pms::pr2-arms-pm
;;                                    pr2-pms::pr2-grippers-pm pr2-pms::pr2-ptu-pm)
;;      (cpl:top-level
;;        ,@body)))

(defun spawn-objects-on-sink-counter (&optional (spawning-poses *object-spawning-poses*))
  (btr-utils:kill-all-objects)
  (add-objects-to-mesh-list)
  (btr:detach-all-objects (btr:get-robot-object))
  (let ((object-types '(:cereal :cup :bowl :spoon :milk)))
    ;; spawn objects at default poses
    (let ((objects (mapcar (lambda (object-type)
                             (btr-utils:spawn-object
                              (intern (format nil "~a-1" object-type) :keyword)
                              object-type
                              :pose (cdr (assoc object-type spawning-poses))))
                           object-types)))
      ;; stabilize world
      (btr:simulate btr:*current-bullet-world* 100)
      objects)))

(defun go-to-sink-or-island (&optional (sink-or-island :sink))
  (let ((?navigation-goal (ecase sink-or-island
                            (:sink *sink-nav-goal*)
                            (:island *island-nav-goal*)))
        (?ptu-goal *look-goal*))
    (cpl:par
      (pp-plans::park-arms)
      (exe:perform (desig:a motion
                            (type going)
                            (target (desig:a location (pose ?navigation-goal))))))
    (exe:perform (desig:a motion
                          (type looking)
                          (target (desig:a location (pose ?ptu-goal)))))))

(defun pick-object (&optional (?object-type :cereal) (?arm :right))
  (pp-plans:park-arms)
  (go-to-sink-or-island :sink)
  (let* ((?object-desig (desig:an object (type ?object-type)))
         (?perceived-object-desig (pp-plans::perceive ?object-desig)))
    (cpl:par
      (exe:perform (desig:an action
                             (type looking)
                             (object ?perceived-object-desig)))
      (exe:perform (desig:an action
                             (type picking-up)
                             (arm ?arm)
                             (object ?perceived-object-desig))))))

(defun place-object (?target-pose &optional (?arm :right))
  (pp-plans:park-arms)
  (go-to-sink-or-island :island)
  (cpl:par
    (exe:perform (desig:a motion
                          (type looking)
                          (target (desig:a location
                                           (pose ?target-pose)))))
    (exe:perform (desig:an action
                           (type placing)
                           (arm ?arm)
                           (target (desig:a location
                                            (pose ?target-pose)))))))

(defun search-for-object (?object-designator ?search-location)
  (cpl:with-retry-counters ((search-location-retries 5))
    (cpl:with-failure-handling
        ((common-fail:perception-object-not-found (e)
           (roslisp:ros-warn (pp-plans search-for-object) "Failure happened: ~a" e)
           (cpl:do-retry search-location-retries
             (setf ?search-location (desig:next-solution ?search-location))
             (when ?search-location
               (roslisp:ros-warn (pp-plans search-for-object) "Retrying...~%")
               (cpl:retry)))))
      (let* ((?pose-at-search-location (desig:reference ?search-location))
             (?nav-location (desig:a location
                                     (visible-for pr2)
                                     (location (desig:a location
                                                        (pose ?pose-at-search-location))))))
        (let ((?pose-at-nav-location (desig:reference ?nav-location)))
          (pp-plans:park-arms)
          (exe:perform (desig:an action
                                 (type going)
                                 (target (desig:a location
                                                  (pose ?pose-at-nav-location))))))
        (exe:perform (desig:an action
                               (type looking)
                               (target (desig:a location
                                                (pose ?pose-at-search-location))))))
      (pp-plans::perceive ?object-designator))))

(defvar *obj* nil)

(defun fetch (?object-designator ?search-location ?arm)
  (let* ((?perceived-object-desig
           (search-for-object ?object-designator ?search-location))
         (?perceived-object-pose-in-base
           (desig:reference (desig:a location (of ?perceived-object-desig))))
         (?perceived-object-pose-in-map
           (cram-tf:ensure-pose-in-frame
            ?perceived-object-pose-in-base
            cram-tf:*fixed-frame*
            :use-zero-time t)))
    (roslisp:ros-info (pp-plans fetch) "Found object ~a" ?perceived-object-desig)

    (let ((?pick-up-location (desig:a location
                                      (reachable-for pr2)
                                      (location (desig:a location
                                                         (pose ?perceived-object-pose-in-map))))))

      (let* ((world btr:*current-bullet-world*)
             (world-state (btr::get-state world)))

        (unwind-protect
             (cpl:with-retry-counters ((reachable-location-retries 21))
               (cpl:with-failure-handling
                   ((common-fail:navigation-pose-in-collision (e)
                      (roslisp:ros-warn (pp-plans fetch) "Failure happened: ~a" e)
                      (cpl:do-retry reachable-location-retries
                        (setf ?pick-up-location (desig:next-solution ?pick-up-location))
                        (when ?pick-up-location
                          (roslisp:ros-warn (pp-plans fetch) "Retrying...~%")
                          (cpl:retry)))))

                 (let ((pose-at-pick-up-location (desig:reference ?pick-up-location)))
                   (pr2-proj::drive pose-at-pick-up-location)
                   (when (btr:find-objects-in-contact
                          btr:*current-bullet-world* (btr:get-robot-object))
                     (roslisp:ros-warn (pp-plans fetch) "Pose was in collision.")
                     (cpl:sleep 0.1)
                     (cpl:fail 'common-fail:navigation-pose-in-collision
                               :pose-stamped pose-at-pick-up-location))
                   (roslisp:ros-info (pp-plans fetch) "Found reachable pose.")
                   (print (btr:find-objects-in-contact
                            btr:*current-bullet-world* (btr:get-robot-object))))))
          (btr::restore-world-state world-state world)))

      (let ((?pose-at-pick-up-location (desig:reference ?pick-up-location)))
        (exe:perform (desig:an action
                               (type going)
                               (target (desig:a location
                                                (pose ?pose-at-pick-up-location)))))
        (exe:perform (desig:an action
                               (type looking)
                               (target (desig:a location
                                                (pose ?perceived-object-pose-in-map))))))))

  (let ((?more-precise-perceived-object-desig
                (pp-plans::perceive ?object-designator)))
          (exe:perform (desig:an action
                                 (type picking-up)
                                 (object ?more-precise-perceived-object-desig)
                                 (arm ?arm)))
          (pp-plans:park-arms)
          (setf *obj* ?more-precise-perceived-object-desig)
          ?more-precise-perceived-object-desig))

(defun deliver (?object-designator ?target-location ?arm)
  (let* ((?pose-at-target-location (desig:reference ?target-location))
         (?nav-location (desig:a location
                                 (reachable-for pr2)
                                 (location (desig:a location
                                                    (pose ?pose-at-target-location))))))
        (let ((?pose-at-nav-location (desig:reference ?nav-location)))
          (pp-plans:park-arms)
          (exe:perform (desig:an action
                                 (type going)
                                 (target (desig:a location
                                                  (pose ?pose-at-nav-location)))))
          (exe:perform (desig:an action
                               (type looking)
                               (target (desig:a location
                                                (pose ?pose-at-target-location)))))
          (exe:perform (desig:an action
                                 (type placing)
                                 (object ?object-designator)
                                 (arm ?arm)
                                 (target (desig:a location
                                                  (pose ?pose-at-target-location))))))))

(defun demo-hard-coded ()
  (spawn-objects-on-sink-counter)

  (with-simulated-robot

    (dolist (object-type '(:cereal :cup :bowl :spoon :milk))

      (let ((placing-target
              (cl-transforms-stamped:pose->pose-stamped
               "map" 0.0
               (cram-bullet-reasoning:ensure-pose
                (cdr (assoc object-type *object-placing-poses*)))))
            (arm-to-use
              (cdr (assoc object-type *object-grasping-arms*))))

        (pick-object object-type arm-to-use)
        (place-object placing-target arm-to-use)))))

(defun demo-random ()
  (spawn-objects-on-sink-counter)
  (setf roslisp::*debug-levels* (make-hash-table :test #'equal))
  (setf cram-robot-pose-guassian-costmap::*orientation-samples* 3)

  (let ((list-of-objects '(:cereal :cup :bowl :spoon :milk)))
    (let* ((short-list-of-objects (remove (nth (random (length list-of-objects))
                                               list-of-objects)
                                          list-of-objects)))
      (setf short-list-of-objects (remove (nth (random (length short-list-of-objects))
                                              short-list-of-objects)
                                          short-list-of-objects))

      (with-simulated-robot
        (dolist (?object-type list-of-objects)
          (let ((?placing-target-pose
                  (cl-transforms-stamped:pose->pose-stamped
                   "map" 0.0
                   (cram-bullet-reasoning:ensure-pose
                    (cdr (assoc ?object-type *object-placing-poses*)))))
                (?arm-to-use
                  (cdr (assoc ?object-type *object-grasping-arms*))))

            (let ((?object
                    (fetch (desig:an object
                                     (type ?object-type))
                           (desig:a location
                                    (on "CounterTop")
                                    (name "iai_kitchen_sink_area_counter_top"))
                           ?arm-to-use)))
              (deliver ?object
                       (desig:a location
                                (pose ?placing-target-pose))
                       ?arm-to-use))))))))
