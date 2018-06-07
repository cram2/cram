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

(in-package :pr2-fd-plans)

(cpl:def-cram-function go-without-collisions (?navigation-location)
  (pp-plans:park-arms)

  (cpl:with-failure-handling
      (((or common-fail:navigation-low-level-failure
            common-fail:actionlib-action-timed-out) (e)
         (roslisp:ros-warn (pp-plans navigate)
                           "Low-level navigation failed: ~a~%.Ignoring anyway." e)
         (return)
         ;; (roslisp:ros-warn (pp-plans navigate)
         ;;                   "Navigation failed: ~a~%.Assuming pose in collision." e)
         ;; (cpl:fail 'common-fail:navigation-pose-in-collision)
         ))

    (pr2-proj-reasoning:check-navigating-collisions ?navigation-location)
    (setf ?navigation-location (desig:current-desig ?navigation-location))
    (exe:perform (desig:an action
                           (type going)
                           (target ?navigation-location)))))


(cpl:def-cram-function turn-towards (?look-target)
  (flet ((calculate-navigation-goal-towards-target (look-pose-stamped robot-pose-stamped)
           "Given a `look-pose-stamped' and a `robot-pose-stamped',
calculate the new robot-pose-stamped, which is rotate with an angle to point towards
the `look-pose-stamped'."
           (let* ((world->robot-transform
                    (cram-tf:pose-stamped->transform-stamped robot-pose-stamped "robot"))
                  (robot->world-transform
                    (cl-transforms:transform-inv world->robot-transform))
                  (world->look-pose-origin
                    (cl-transforms:origin look-pose-stamped))
                  (look-pose-in-robot-frame
                    (cl-transforms:transform-point
                     robot->world-transform
                     world->look-pose-origin))
                  (rotation-angle
                    (atan
                     (cl-transforms:y look-pose-in-robot-frame)
                     (cl-transforms:x look-pose-in-robot-frame))))
             (cram-tf:rotate-pose robot-pose-stamped :z rotation-angle))))

    (let (?navigation-goal)
      (cpl:with-retry-counters ((turn-around-retries 1))
        (cpl:with-failure-handling
            (((or common-fail:ptu-goal-unreachable) (e)
               (roslisp:ros-warn (pp-plans turn-towards)
                                 "PTU action wanted to twist the neck~%.~a~%Turning around." e)
               (cpl:do-retry turn-around-retries
                 (let (look-target-pose)
                   (handler-case
                       (setf look-target-pose (desig:reference ?look-target))
                     (desig:designator-error ()
                       (roslisp:ros-warn (pp-plans turn-towards)
                                         "Designator cannot be resolved: ~a. Propagating up."
                                         ?look-target)
                       (cpl:fail 'common-fail:high-level-failure)))
                   (setf ?navigation-goal
                         (calculate-navigation-goal-towards-target
                          look-target-pose
                          (cram-tf:robot-current-pose))))
                 (cpl:retry))
               (roslisp:ros-warn (pp-plans turn-towards) "Turning around didn't work :'(~%")
               (cpl:fail 'common-fail:high-level-failure)))

          (when ?navigation-goal
            (exe:perform (desig:an action
                                   (type navigating)
                                   (location (desig:a location
                                                      (pose ?navigation-goal))))))
          (exe:perform (desig:an action
                                 (type looking)
                                 (target ?look-target))))))))


(cpl:def-cram-function manipulate-environment (?object-to-access ?arm)
  (exe:perform (desig:an action
                         (type navigating)
                         (location (desig:a location
                                            (reachable-for pr2)
                                            (arm ?arm)
                                            (object ?object-to-access)))))
  (let ((offset cram-pr2-environment-manipulation::*drawer-handle-lift-x-offset*))
    (setf cram-pr2-environment-manipulation::*drawer-handle-lift-x-offset* 0.10)
    (exe:perform (desig:an action
                           (type opening)
                           (arm ?arm)
                           (object ?object-to-access)))
    (exe:perform (desig:an action
                           (type closing)
                           (arm ?arm)
                           (object ?object-to-access)))
    (setf cram-pr2-environment-manipulation::*drawer-handle-lift-x-offset* offset)))


(cpl:def-cram-function search-for-object (?object-designator ?search-location location-accessible
                                                             &optional (retries 4))
  "Searches for `?object-designator' in its likely location `?search-location'."
  (unless location-accessible
    (exe:perform (desig:an action
                           (type accessing)
                           (location ?search-location))))

  ;; take new `?search-location' sample if a failure happens and retry
  (cpl:with-retry-counters ((search-location-retries retries))
    (cpl:with-failure-handling
        (((or common-fail:perception-low-level-failure
              common-fail:navigation-pose-in-collision) (e)
           (roslisp:ros-warn (pp-plans search-for-object) "~a" e)
           (cpl:do-retry search-location-retries
             (handler-case
                 (setf ?search-location (desig:next-solution ?search-location))
               (desig:designator-error ()
                 (roslisp:ros-warn (pp-plans search-for-object)
                                   "Designator cannot be resolved: ~a. Propagating up." e)
                 (cpl:fail 'common-fail:object-nowhere-to-be-found)))
             (if ?search-location
                 (progn
                   (roslisp:ros-warn (pp-plans search-for-object) "Retrying...~%")
                   (cpl:retry))
                 (progn
                   (roslisp:ros-warn (pp-plans search-for-object) "No samples left :'(~%")
                   (cpl:fail 'common-fail:object-nowhere-to-be-found))))
           (roslisp:ros-warn (pp-plans search-for-object) "No retries left :'(~%")
           (cpl:fail 'common-fail:object-nowhere-to-be-found)))

      ;; navigate, look and detect
      (let* ((?pose-at-search-location (desig:reference ?search-location))
             (?nav-location (desig:a location
                                     (visible-for pr2)
                                     (location (desig:a location
                                                        (pose ?pose-at-search-location))))))
        (exe:perform (desig:an action
                               (type navigating)
                               (location ?nav-location)))
        (exe:perform (desig:an action
                               (type turning-towards)
                               (target (desig:a location
                                                (pose ?pose-at-search-location)))))
        (exe:perform (desig:an action
                               (type detecting)
                               (object ?object-designator)))))))



(cpl:def-cram-function fetch (?object-designator ?arm ?pick-up-robot-location pick-up-action)

  (let* ((?object-pose-in-map
           (desig:reference (desig:a location (of-item-object ?object-designator))))
         (?pick-up-robot-location
            (or (desig:current-desig ?pick-up-robot-location)
                (desig:a location
                         (reachable-for pr2)
                         (location (desig:a location
                                            (pose ?object-pose-in-map)))))))

    ;; take a new `?pick-up-robot-location' sample if a failure happens
    (cpl:with-retry-counters ((relocation-for-ik-retries 20))
      (cpl:with-failure-handling
          (((or common-fail:object-unreachable
                common-fail:perception-low-level-failure
                common-fail:gripping-failed
                common-fail:high-level-failure
                common-fail:navigation-pose-in-collision) (e)
             (declare (ignore e))
             (roslisp:ros-warn (pp-plans fetch)
                               "Object of type ~a is unreachable."
                               (desig:desig-prop-value ?object-designator :type))
             (cpl:do-retry relocation-for-ik-retries
               (handler-case
                   (setf ?pick-up-robot-location (desig:next-solution ?pick-up-robot-location))
                 (desig:designator-error ()
                   (roslisp:ros-warn (pp-plans fetch)
                                     "Designator to reach object ~a cannot be resolved. ~
                                          Propagating up."
                                     (desig:desig-prop-value ?object-designator :type))
                   (cpl:fail 'common-fail:object-unfetchable :object ?object-designator)))
               (if ?pick-up-robot-location
                   (progn
                     (roslisp:ros-info (pp-plans fetch) "Relocating...")
                     (cpl:retry))
                   (progn
                     (roslisp:ros-warn (pp-plans fetch) "No more samples to try :'(")
                     (cpl:fail 'common-fail:object-unfetchable))))
             (roslisp:ros-warn (pp-plans fetch) "No more retries left :'(")
             (cpl:fail 'common-fail:object-unfetchable :object ?object-designator)))

        ;; navigate, look, detect and pick-up
        (exe:perform (desig:an action
                               (type navigating)
                               (location ?pick-up-robot-location)))
        (setf ?pick-up-robot-location (desig:current-desig ?pick-up-robot-location))

        (exe:perform (desig:an action
                               (type turning-towards)
                               (target (desig:a location
                                                (pose ?object-pose-in-map)))))

        (let ((?more-precise-perceived-object-desig
                (exe:perform (desig:an action
                                       (type detecting)
                                       (object ?object-designator)))))

          (let ((pick-up-action
                  ;; if pick-up-action already exists, use its params for picking up
                  (or (when pick-up-action
                        (destructuring-bind (_action _object-designator ?arm
                                             _gripper-opening _effort ?grasp
                                             _left-reach-poses _right-reach-poses
                                             _left-lift-poses _right-lift-poses)
                            (desig:reference pick-up-action)
                          (desig:an action
                                    (type picking-up)
                                    (arm ?arm)
                                    (grasp ?grasp)
                                    (object ?more-precise-perceived-object-desig))))
                      (desig:an action
                                (type picking-up)
                                (desig:when ?arm
                                  (arm ?arm))
                                (object ?more-precise-perceived-object-desig)))))

            (setf pick-up-action (desig:current-desig pick-up-action))
            (pr2-proj-reasoning:check-picking-up-collisions pick-up-action)
            (setf pick-up-action (desig:current-desig pick-up-action))

            (exe:perform pick-up-action)))))

    (pp-plans:park-arms)
    (desig:current-desig ?object-designator)))


(cpl:def-cram-function deliver (?object-designator ?target-location
                                                   ?target-robot-location place-action)

  ;; Reference the `?target-location' to see if that works at all
  ;; If not, delivering is impossible so throw a OBJECT-UNDERLIVERABLE failure
  (let* (?pose-at-target-location ?target-robot-location)
    (handler-case
        (setf ?pose-at-target-location (desig:reference ?target-location))
      (desig:designator-error ()
        (roslisp:ros-warn (pp-plans deliver)
                          "Designator cannot be resolved: ~a. Propagating up."
                          ?target-location)
        (cpl:fail 'common-fail:object-undeliverable)))
    (setf ?target-robot-location
          (or (desig:current-desig ?target-robot-location)
              (desig:a location
                       (reachable-for pr2)
                       (location (desig:a location
                                          (pose ?pose-at-target-location))))))

    ;; take a new `?target-robot-location' sample if a failure happens
    (cpl:with-retry-counters ((relocation-for-ik-retries 5))
      (cpl:with-failure-handling
          (((or common-fail:manipulation-pose-in-collision
                common-fail:navigation-pose-in-collision) (e)
             (roslisp:ros-warn (pp-plans deliver)
                               "Object is undeliverable from current base location.~%~a" e)
             (cpl:do-retry relocation-for-ik-retries
               (handler-case
                   (setf ?target-robot-location
                         (desig:next-solution ?target-robot-location))
                 (desig:designator-error ()
                   (roslisp:ros-warn (pp-plans coll-check)
                                     "Designator cannot be resolved: ~a.~
                                      Propagating up." e)
                   (cpl:fail 'common-fail:object-undeliverable)))
               (if ?target-robot-location
                   (progn
                     (roslisp:ros-info (pp-plans deliver) "Relocating...")
                     (cpl:retry))
                   (progn
                     (roslisp:ros-warn (pp-plans deliver) "No more samples to try :'(")
                     (cpl:fail 'common-fail:object-undeliverable))))))

        ;; navigate
        (exe:perform (desig:an action
                               (type navigating)
                               (location ?target-robot-location)))
        (setf ?target-robot-location (desig:current-desig ?target-robot-location))

        (unless
            ;; take a new `?target-location' sample if a failure happens
            (cpl:with-retry-counters ((target-location-retries 10))
              (cpl:with-failure-handling
                  (((or common-fail:object-unreachable
                        common-fail:manipulation-pose-unreachable
                        common-fail:manipulation-pose-in-collision) (e)
                     (roslisp:ros-warn (pp-plans deliver) "Failure happened: ~a" e)
                     (cpl:do-retry target-location-retries
                       (handler-case
                           (setf ?target-location (desig:next-solution ?target-location))
                         (desig:designator-error ()
                           (roslisp:ros-warn (pp-plans deliver)
                                             "Designator cannot be resolved: ~a. Propagating up."
                                             ?target-location)
                           (cpl:fail 'common-fail:object-undeliverable)))
                       (if ?target-location
                           (progn
                             (roslisp:ros-warn (pp-plans deliver) "Retrying...~%")
                             (cpl:retry))
                           (progn
                             (roslisp:ros-warn (pp-plans deliver) "No samples left :'(~%")
                             (cpl:fail 'common-fail:object-undeliverable))))
                     (roslisp:ros-warn (pp-plans deliver) "No target-location-retries left :'(~%")
                     (cpl:fail 'common-fail:manipulation-pose-in-collision)
                     (return)))

                ;; look, place
                (let* ((?pose-at-target-location (desig:reference ?target-location))
                       (mTt
                         (cram-tf:pose-stamped->transform-stamped
                          ?pose-at-target-location
                          "placing_target_location")) ; bTt = bTm * mTt = (mTb)-1 * mTt
                       (mTb
                         (cram-tf:pose->transform-stamped
                          cram-tf:*fixed-frame*
                          cram-tf:*robot-base-frame*
                          0.0
                          (btr:pose (btr:get-robot-object))))
                       (mTb-1
                         (cram-tf:transform-stamped-inv
                          mTb))
                       (bTt
                         (cram-tf:multiply-transform-stampeds
                          cram-tf:*robot-base-frame*
                          "placing_target_location"
                          mTb-1 mTt
                          :result-as-pose-or-transform :pose))
                       (?pose-at-target-loc-in-base
                         bTt))

                  (exe:perform (desig:an action
                                         (type turning-towards)
                                         (target (desig:a location
                                                          (pose ?pose-at-target-loc-in-base)))))

                  (let ((place-action
                          (or
                           ;; TODO: maybe pass the placing action as well?
                           ;; place-action
                           (desig:an action
                                     (type placing)
                                     (object ?object-designator)
                                     (target (desig:a location
                                                      (pose ?pose-at-target-loc-in-base)))))))

                    (setf place-action (desig:current-desig place-action))
                    (pr2-proj-reasoning:check-placing-collisions place-action)
                    (setf place-action (desig:current-desig place-action))

                    (exe:perform place-action)
                    T))))

          (roslisp:ros-warn (pp-plans deliver) "No relocation-for-ik-retries left :'(")
          (cpl:fail 'common-fail:object-undeliverable))))))



(defun drop-at-sink ()
  (let ((?map-in-front-of-sink-pose
          (cl-transforms-stamped:make-pose-stamped
           cram-tf:*fixed-frame*
           0.0
           (cl-transforms:make-3d-vector 0.7 -0.4 0)
           (cl-transforms:make-identity-rotation)))
        (?placing-pose
          (cl-transforms-stamped:make-pose-stamped
           cram-tf:*robot-base-frame*
           0.0
           (cl-transforms:make-3d-vector 0.7 0 1.2)
           (cl-transforms:make-identity-rotation))))
    (cpl:with-failure-handling
        ((common-fail:navigation-low-level-failure (e)
           (declare (ignore e))
           (return)))
      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location
                                  (pose ?map-in-front-of-sink-pose))))))
    (cpl:with-failure-handling
        ((cpl:plan-failure (e)
           (declare (ignore e))
           (return)))
      (exe:perform
       (desig:an action
                 (type placing)
                 (target (desig:a location
                                  (pose ?placing-pose))))))))


(cpl:def-cram-function transport (?object-designator ?search-location ?delivering-location ?arm)
  (let ((?perceived-object-designator
          (exe:perform (desig:an action
                                 (type searching)
                                 (object ?object-designator)
                                 (location ?search-location)))))
    (roslisp:ros-info (pp-plans transport)
                      "Found object of type ~a."
                      (desig:desig-prop-value ?perceived-object-designator :type))

    (let (?fetch-robot-location
          ?fetch-pick-up-action
          ?deliver-robot-location
          ?deliver-place-action)

      ;; If running on the real robot, execute below task tree in projection
      ;; N times first, then pick the best parameterization
      ;; and use that parameterization in the real world.
      ;; If running in projection, just execute the task tree below as normal.
      (pr2-proj-reasoning:with-projected-task-tree
          (?fetch-robot-location ?fetch-pick-up-action
           ?deliver-robot-location ?deliver-place-action)
          4
          #'pr2-proj-reasoning:pick-best-parameters-by-distance

        (let ((?fetched-object
                (exe:perform (desig:an action
                                       (type fetching)
                                       (when ?arm
                                         (arm ?arm))
                                       (object ?perceived-object-designator)
                                       (robot-location ?fetch-robot-location)
                                       (pick-up-action ?fetch-pick-up-action)))))
          (roslisp:ros-info (pp-plans transport) "Fetched the object.")

          (cpl:with-failure-handling
              ((common-fail:high-level-failure (e)
                 (declare (ignore e))
                 (drop-at-sink)
                 (return)))
            (exe:perform (desig:an action
                                   (type delivering)
                                   (object ?fetched-object)
                                   (target ?delivering-location)
                                   (robot-location ?deliver-robot-location)
                                   (place-action ?deliver-place-action)))))))))
