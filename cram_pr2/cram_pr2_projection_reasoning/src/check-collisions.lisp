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

(defun check-navigating-collisions (navigation-location-desig &optional (samples-to-try 30))
  (declare (type desig:location-designator navigation-location-desig))
  "Store current world state and in the current world try to go to different
poses that satisfy `navigation-location-desig'.
If chosen pose results in collisions, choose another pose.
Repeat `navigation-location-samples' + 1 times.
Store found pose into designator or throw error if good pose not found."

  (cpl:with-failure-handling
      ((desig:designator-error (e)
         (roslisp:ros-warn (coll-check nav)
                           "Desig ~a could not be resolved: ~a~%Cannot navigate."
                           navigation-location-desig e)
         (cpl:fail 'common-fail:navigation-goal-in-collision
                   :description "Designator could not be resolved")))

    (cpl:with-retry-counters ((navigation-location-samples samples-to-try))
      ;; If a navigation-low-level-failure happens, retry N times
      ;; with the next solution of `navigation-location-desig'.
      (cpl:with-failure-handling
          ((common-fail:navigation-low-level-failure (e)
             (declare (ignore e))
             (roslisp:ros-warn (coll-check nav) "Pose was in collision.")
             (cpl:do-retry navigation-location-samples
               (if (desig:next-solution navigation-location-desig)
                   (progn
                     (setf navigation-location-desig
                           (desig:next-solution navigation-location-desig))
                     (cpl:retry))
                   (progn
                     (roslisp:ros-warn (coll-check nav)
                                       "No other samples in designator. Propagating up.")
                     (cpl:fail 'common-fail:navigation-goal-in-collision
                               :description "No other samples in designator"))))
             (roslisp:ros-warn (coll-check nav)
                               "Couldn't find a nav pose after all retries for~%~a.~%~
                                     Propagating up."
                               navigation-location-desig)
             (cpl:fail 'common-fail:navigation-goal-in-collision
                       :description "Couldn't find a nav pose after all retries")))

        ;; Pick one pose, store it in `pose-at-navigation-location'
        ;; In projected world, drive to picked pose
        ;; If robot is in collision with any object in the world, driving throws a failure.
        ;; Otherwise, the pose was found, so return location designator,
        ;; which is currently referenced to the found pose.
        (let ((pose-at-navigation-location (desig:reference navigation-location-desig)))
          (pr2-proj::drive pose-at-navigation-location)
          ;; (roslisp:ros-info (coll-check nav)
          ;;                   "Found non-colliding pose~%~a to satisfy~%~a."
          ;;                   pose-at-navigation-location navigation-location-desig)
          navigation-location-desig)))))




(defun check-picking-up-collisions (pick-up-action-desig &optional (retries 30))
  (let* ((world btr:*current-bullet-world*)
         (world-state (btr::get-state world)))

    (unwind-protect
         (cpl:with-failure-handling
             ((desig:designator-error (e)
                (roslisp:ros-warn (coll-check pick)
                                  "Desig ~a could not be resolved: ~a~%Cannot pick."
                                  pick-up-action-desig e)
                (cpl:fail 'common-fail:object-unreachable
                          :description "Designator could not be resolved")))

           ;; When the pick-up goal configuration is unreachable based on projection
           ;; or if resulting configuration collides with environment,
           ;; take a new configuration and retry.
           ;; If no configuration is good, throw `object-unreachable' failure
           (cpl:with-retry-counters ((pick-up-configuration-retries retries))
             (cpl:with-failure-handling
                 (((or common-fail:manipulation-pose-unreachable
                       common-fail:manipulation-goal-in-collision) (e)
                    (declare (ignore e))
                    (cpl:do-retry pick-up-configuration-retries
                      (setf pick-up-action-desig
                            (desig:next-solution pick-up-action-desig))
                      (if pick-up-action-desig
                          (cpl:retry)
                          (progn
                            (roslisp:ros-warn (coll-check pick)
                                              "No more pick-up samples to try.~
                                               Object unreachable.")
                            (cpl:fail 'common-fail:object-unreachable
                                      :description "No more pick-up samples to try."))))
                    (roslisp:ros-warn (coll-check pick) "No more retries left :'(")
                    (cpl:fail 'common-fail:object-unreachable
                              :description "No more grasp retries left.")))

               (let ((pick-up-action-referenced (desig:reference pick-up-action-desig)))
                 (destructuring-bind (_action object-designator arm gripper-opening _effort _grasp
                                      left-reach-poses right-reach-poses
                                      left-grasp-poses right-grasp-poses
                                      left-lift-poses right-lift-poses)
                     pick-up-action-referenced
                   (declare (ignore _action _effort))

                   (pr2-proj::gripper-action gripper-opening arm)

                   ;; Go over all the trajectory via points and check for collisions
                   ;; with any object except the one to pick up.
                   ;; If collision happens, throw `manipulation-goal-in-collision' failure.
                   (let ((object-name
                           (desig:desig-prop-value object-designator :name)))
                     (roslisp:ros-info (coll-check pick)
                                       "Trying grasp ~a on object ~a with arm ~a~%"
                                       _grasp object-name arm)
                     (let ((left-poses-list-of-lists (list left-reach-poses
                                                           left-grasp-poses
                                                           left-lift-poses))
                           (right-poses-list-of-lists (list right-reach-poses
                                                            right-grasp-poses
                                                            right-lift-poses)))
                       (multiple-value-bind (left-poses right-poses)
                           (cut:equalize-lists-of-lists-lengths left-poses-list-of-lists
                                                                right-poses-list-of-lists)
                         (mapcar (lambda (left-pose right-pose)
                                   (pr2-proj::move-tcp left-pose right-pose :allow-all)
                                   (unless (< (abs pr2-proj:*debug-short-sleep-duration*) 0.0001)
                                     (cpl:sleep pr2-proj:*debug-short-sleep-duration*))
                                   (when (remove object-name
                                                 (btr:robot-colliding-objects-without-attached))
                                     (roslisp:ros-warn (coll-check pick)
                                                       "Robot is in collision with environment.")
                                     (cpl:sleep pr2-proj:*debug-long-sleep-duration*)
                                     (btr::restore-world-state world-state world)
                                     (cpl:fail 'common-fail:manipulation-goal-in-collision)))
                                 left-poses
                                 right-poses)))))))))
      (btr::restore-world-state world-state world))))



(defun check-placing-collisions (placing-action-desig)
  (let* ((world btr:*current-bullet-world*)
         (world-state (btr::get-state world)))
    (unwind-protect
         (cpl:with-failure-handling
             ((desig:designator-error (e)
                (roslisp:ros-warn (coll-check place)
                                  "Desig ~a could not be resolved: ~a~%Cannot pick."
                                  placing-action-desig e)
                (cpl:fail 'common-fail:object-unreachable
                          :description "Designator could not be resolved"))

              ((or common-fail:manipulation-goal-in-collision
                   common-fail:manipulation-low-level-failure) (e)
                (declare (ignore e))
                (roslisp:ros-warn (coll-check place)
                                  "Placing pose of ~a is unreachable.~%Propagating up."
                                  placing-action-desig)
                (cpl:fail 'common-fail:object-unreachable)))

           (let ((placing-action-referenced (desig:reference placing-action-desig)))
             (destructuring-bind (_action object-designator on-object-designator
                                  _assemblage-name
                                  arm
                                  _gripper-opening
                                  left-reach-poses right-reach-poses
                                  left-put-poses right-put-poses
                                  left-retract-poses right-retract-poses
                                  _placing-location)
                 placing-action-referenced
               (declare (ignore _action _assemblage-name _gripper-opening _placing-location))

               (pr2-proj::gripper-action :open arm)

               (let ((object-name (desig:desig-prop-value object-designator :name)))
                 (roslisp:ros-info (coll-check place)
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
                              (pr2-proj::move-tcp left-pose right-pose :allow-all)
                              (unless (< (abs pr2-proj:*debug-short-sleep-duration*) 0.0001)
                                (cpl:sleep pr2-proj:*debug-short-sleep-duration*))
                              (when (or
                                     ;; either robot collides with environment
                                     (btr:robot-colliding-objects-without-attached)
                                     ;; or object in the hand collides with environment
                                     ;; (remove (btr:name
                                     ;;          (find-if (lambda (x)
                                     ;;                     (typep x 'btr:semantic-map-object))
                                     ;;                   (btr:objects btr:*current-bullet-world*)))
                                     ;;         (remove (btr:get-robot-name)
                                     ;;                 (btr:find-objects-in-contact
                                     ;;                  btr:*current-bullet-world*
                                     ;;                  (btr:object
                                     ;;                   btr:*current-bullet-world*
                                     ;;                   object-name))
                                     ;;                 :key #'btr:name)
                                     ;;         :key #'btr:name)
                                     )
                                (roslisp:ros-warn (coll-check place)
                                                  "Robot is in collision with environment.")
                                (cpl:sleep pr2-proj:*debug-long-sleep-duration*)
                                (btr::restore-world-state world-state world)
                                ;; (cpl:fail 'common-fail:manipulation-goal-in-collision)
                                ))
                            left-poses
                            right-poses)))))))
      (btr::restore-world-state world-state world))))



(defun check-placing-pose-stability (object-desig placing-location)
  (let* ((placing-pose
           (desig:reference placing-location))
         (world
           btr:*current-bullet-world*)
         (world-state
           (btr::get-state world))
         (bullet-object-type
           (desig:desig-prop-value object-desig :type))
         (new-btr-object
           (btr-utils:spawn-object
            (gensym "obj") bullet-object-type :pose placing-pose)))
    (unwind-protect
         (progn
           (setf (btr:pose new-btr-object) placing-pose)
           (cpl:sleep pr2-proj::*debug-short-sleep-duration*)
           (btr:simulate btr:*current-bullet-world* 500)
           (btr:simulate btr:*current-bullet-world* 100)
           (let* ((new-pose
                    (btr:pose new-btr-object))
                  (distance-new-pose-and-place-pose
                    (cl-tf:v-dist
                     (cl-transforms:origin new-pose)
                     (cl-transforms:origin placing-pose))))
             (when (> distance-new-pose-and-place-pose 0.2)
               (cpl:fail 'common-fail:manipulation-low-level-failure
                         :description "Pose unstable."))))
      (btr::restore-world-state world-state world))))



(defun check-environment-manipulation-collisions (action-desig)
  (let* ((world btr:*current-bullet-world*)
         (world-state (btr::get-state world)))
    (unwind-protect
         (cpl:with-failure-handling
             ((desig:designator-error (e)
                (roslisp:ros-warn (coll-check environment)
                                  "Desig ~a could not be resolved: ~a~%Cannot manipulate."
                                  action-desig e)
                (cpl:fail 'common-fail:environment-unreachable
                          :description "Designator could not be resolved"))

              ((or common-fail:manipulation-goal-in-collision
                   common-fail:manipulation-low-level-failure) (e)
                (declare (ignore e))
                (roslisp:ros-warn (coll-check environment)
                                  "Manipulation pose of ~a is unreachable or colliding.~%~
                                   Propagating up."
                                  action-desig)
                (cpl:fail 'common-fail:environment-unreachable
                          :description "Manipulation pose in collision or unreachable.")))

           (let ((action-referenced (desig:reference action-desig)))
             (destructuring-bind (action arm _gripper-opening distance
                                  left-reach-poses right-reach-poses
                                  left-grasp-poses right-grasp-poses
                                  left-pull-push-poses right-pull-push-poses
                                  left-retract-poses right-retract-poses
                                  joint-name link-name _environment-object)
                 action-referenced
               (declare (ignore _gripper-opening _environment-object))

               (pr2-proj::gripper-action :open arm)

               (roslisp:ros-info (coll-check environment)
                                 "Trying to ~a with joint ~a with arm ~a~%"
                                 action joint-name arm)
               (let ((left-poses-list-of-lists
                       (list left-reach-poses left-grasp-poses
                             left-pull-push-poses left-retract-poses))
                     (right-poses-list-of-lists
                       (list right-reach-poses right-grasp-poses
                             right-pull-push-poses right-retract-poses)))
                 (multiple-value-bind (left-poses right-poses)
                     (cut:equalize-lists-of-lists-lengths left-poses-list-of-lists
                                                          right-poses-list-of-lists)
                   (mapcar (lambda (left-pose right-pose)
                             (pr2-proj::move-tcp left-pose right-pose :allow-all)
                             (unless (< (abs pr2-proj:*debug-short-sleep-duration*) 0.0001)
                               (cpl:sleep pr2-proj:*debug-short-sleep-duration*)))
                           left-poses
                           right-poses)
                   (when (eq (desig:desig-prop-value action-desig :type) :opening)
                     (when (btr:robot-colliding-objects-without-attached)
                       (roslisp:ros-warn (coll-check environment)
                                         "Robot is in collision with environment.")
                       (cpl:sleep pr2-proj:*debug-long-sleep-duration*)
                       (btr::restore-world-state world-state world)
                       ;; (cpl:fail 'common-fail:manipulation-goal-in-collision)
                       )))))))
      (btr::restore-world-state world-state world))))
