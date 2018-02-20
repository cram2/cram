;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;               2017, Christopher Pollok <cpollok@uni-bremen.de>
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

(defparameter *debug-short-sleep-duration* 0.0 "in seconds")
(defparameter *debug-long-sleep-duration* 0.0 "in seconds")

(defun check-navigating-collisions (navigation-location-desig &optional (samples-to-try 30))
  (declare (type desig:location-designator navigation-location-desig))
  "Store current world state and in the current world try to go to different
poses that satisfy `navigation-location-desig'.
If chosen pose results in collisions, choose another pose.
Repeat `navigation-location-samples' + 1 times.
Store found pose into designator or throw error if good pose not found."
  (let* ((world btr:*current-bullet-world*)
         (world-state (btr::get-state world)))

    (unwind-protect
         (cpl:with-retry-counters ((navigation-location-samples samples-to-try))
           ;; If a navigation-pose-in-collisions failure happens, retry N times
           ;; with the next solution of `navigation-location-desig'.
           (cpl:with-failure-handling
               ((common-fail:navigation-pose-in-collision (e)
                  ;; (roslisp:ros-warn (pp-plans coll-check) "Pose was in collision.")
                  (cpl:do-retry navigation-location-samples
                    (handler-case
                        (setf navigation-location-desig
                              (desig:next-solution navigation-location-desig))
                      (desig:designator-error ()
                        (roslisp:ros-warn (pp-plans coll-check)
                                          "Designator cannot be resolved: ~a. Propagating up." e)
                        (cpl:fail 'common-fail:navigation-pose-in-collision)))
                    (if navigation-location-desig
                        (progn
                          (cpl:retry))
                        (progn
                          (roslisp:ros-warn (pp-plans coll-check)
                                            "No other samples in designator. Propagating up.")
                          (cpl:fail 'common-fail:navigation-pose-in-collision))))
                  (roslisp:ros-warn (pp-plans coll-check)
                                    "Couldn't find a nav pose for~%~a.~%Propagating up."
                                    navigation-location-desig)
                  (cpl:fail 'common-fail:navigation-pose-in-collision)))

             ;; Pick one pose, store it in `pose-at-navigation-location'
             ;; In projected world, drive to picked pose
             ;; If robot is in collision with any object in the world, throw a failure.
             ;; Otherwise, the pose was found, so return location designator,
             ;; which is currently referenced to the found pose.
             (handler-case
                 (let ((pose-at-navigation-location (desig:reference navigation-location-desig)))
                   (pr2-proj::drive pose-at-navigation-location)
                   (when (btr:robot-colliding-objects-without-attached)
                     ;; (roslisp:ros-warn (pp-plans coll-check) "Pose was in collision.")
                     (unless (< (abs *debug-short-sleep-duration*) 0.0001)
                       (cpl:sleep *debug-short-sleep-duration*))
                     (cpl:fail 'common-fail:navigation-pose-in-collision
                               :pose-stamped pose-at-navigation-location))
                   (roslisp:ros-info (pp-plans coll-check)
                                     "Found non-colliding pose to satisfy ~a."
                                     navigation-location-desig)
                   navigation-location-desig)
               (desig:designator-error (e)
                 (declare (ignore e))
                 (roslisp:ros-warn (pp-plans coll-check)
                                   "Desig ~a could not be resolved.~%Cannot navigate."
                                   navigation-location-desig)
                 (cpl:fail 'common-fail:navigation-pose-in-collision)))))

      ;; After playing around and messing up the world, restore the original state.
      (btr::restore-world-state world-state world))))


(defun check-picking-up-collisions (pick-up-action-desig &optional (retries 30))
  (let* ((world btr:*current-bullet-world*)
         (world-state (btr::get-state world)))

    (unwind-protect
         (cpl:with-retry-counters ((pick-up-configuration-retries retries))
           (cpl:with-failure-handling
               (((or common-fail:manipulation-pose-unreachable
                     common-fail:manipulation-pose-in-collision) (e)
                  (declare (ignore e))
                  (cpl:do-retry pick-up-configuration-retries
                    (handler-case
                        (setf pick-up-action-desig
                              (desig:next-solution pick-up-action-desig))
                      (desig:designator-error ()
                        (roslisp:ros-warn (pp-plans coll-check)
                                          "Designator ~a cannot be resolved. Propagating up."
                                          pick-up-action-desig)
                        (cpl:fail 'common-fail:object-unreachable)))
                    (cond
                      (pick-up-action-desig
                       (cpl:retry))
                      (t
                       (roslisp:ros-warn (pp-plans coll-check)
                                         "No more pick-up samples to try. Object unreachable.")
                       (cpl:fail 'common-fail:object-unreachable))))
                  (roslisp:ros-warn (pp-plans pick-object) "No more retries left :'(")
                  (cpl:fail 'common-fail:object-unreachable)))

             (let ((pick-up-action-referenced (desig:reference pick-up-action-desig)))
               (destructuring-bind (_action object-designator arm gripper-opening _effort _grasp
                                    left-reach-poses right-reach-poses
                                    left-lift-poses right-lift-poses)
                   pick-up-action-referenced
                 (declare (ignore _action _effort))
                 (let ((object-name
                         (desig:desig-prop-value object-designator :name)))
                   (roslisp:ros-info (pp-plans manipulation)
                                     "Trying grasp ~a on object ~a with arm ~a~%"
                                     _grasp object-name arm)
                   (let ((left-poses-list-of-lists (list left-reach-poses left-lift-poses))
                         (right-poses-list-of-lists (list right-reach-poses right-lift-poses)))
                     (multiple-value-bind (left-poses right-poses)
                         (cut:equalize-lists-of-lists-lengths left-poses-list-of-lists
                                                              right-poses-list-of-lists)
                       (mapcar (lambda (left-pose right-pose)
                                 (pr2-proj::gripper-action gripper-opening arm)
                                 (pr2-proj::move-tcp left-pose right-pose)
                                 (cram-occasions-events:on-event
                                  (make-instance 'cram-plan-occasions-events:robot-state-changed))
                                 (unless (< (abs *debug-short-sleep-duration*) 0.0001)
                                   (cpl:sleep *debug-short-sleep-duration*))
                                 (when (remove object-name
                                               (btr:robot-colliding-objects-without-attached))
                                   (roslisp:ros-warn (pp-plans coll-check)
                                                     "Robot is in collision with environment.")
                                   (cpl:sleep *debug-long-sleep-duration*)
                                   (btr::restore-world-state world-state world)
                                   (cpl:fail 'common-fail:manipulation-pose-in-collision)))
                               left-poses
                               right-poses))))))))
      (btr::restore-world-state world-state world))))


(defun check-placing-collisions (placing-action-desig)
  (let* ((world btr:*current-bullet-world*)
         (world-state (btr::get-state world)))

    (unwind-protect
         (cpl:with-failure-handling
             ((common-fail:manipulation-pose-unreachable (e)
                (declare (ignore e))
                (roslisp:ros-warn (pp-plans deliver)
                                  "Placing pose of ~a is unreachable.~%Propagating up."
                                  placing-action-desig)
                (cpl:fail 'common-fail:object-unreachable)))

           (let ((placing-action-referenced (desig:reference placing-action-desig)))
             (destructuring-bind (_action object-designator arm
                                  left-reach-poses right-reach-poses
                                  left-put-poses right-put-poses
                                  left-retract-poses right-retract-poses)
                 placing-action-referenced
               (declare (ignore _action))
               (let ((object-name
                       (desig:desig-prop-value object-designator :name)))
                 (roslisp:ros-info (pp-plans manipulation)
                                   "Trying to place object ~a with arm ~a~%"
                                   object-name arm)
                (let ((left-poses-list-of-lists
                        (list left-reach-poses left-put-poses left-retract-poses))
                      (right-poses-list-of-lists
                        (list right-reach-poses right-put-poses right-retract-poses)))
                  (multiple-value-bind (left-poses right-poses)
                      (cut:equalize-lists-of-lists-lengths left-poses-list-of-lists
                                                           right-poses-list-of-lists)
                    (mapcar (lambda (left-pose right-pose)
                              (pr2-proj::gripper-action :open arm)
                              (pr2-proj::move-tcp left-pose right-pose)
                              (cram-occasions-events:on-event
                               (make-instance 'cram-plan-occasions-events:robot-state-changed))
                              (unless (< (abs *debug-short-sleep-duration*) 0.0001)
                                (cpl:sleep *debug-short-sleep-duration*))
                              (when (or
                                     ;; either robot collides with environment
                                     (btr:robot-colliding-objects-without-attached)
                                     ;; or object in the hand collides with environment
                                     (remove (btr:name
                                              (find-if (lambda (x)
                                                         (typep x 'btr:semantic-map-object))
                                                       (btr:objects btr:*current-bullet-world*)))
                                             (remove (btr:get-robot-name)
                                                     (btr:find-objects-in-contact
                                                      btr:*current-bullet-world*
                                                      (btr:object
                                                       btr:*current-bullet-world*
                                                       object-name))
                                                     :key #'btr:name)
                                             :key #'btr:name))
                                (roslisp:ros-warn (pp-plans coll-check)
                                                  "Robot is in collision with environment.")
                                (cpl:sleep *debug-long-sleep-duration*)
                                (btr::restore-world-state world-state world)
                                (cpl:fail 'common-fail:manipulation-pose-in-collision)))
                            left-poses
                            right-poses)))))))
      (btr::restore-world-state world-state world))))
