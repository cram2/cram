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
  (exe:perform (desig:an action
                         (type positioning-arm)
                         (left-configuration park)
                         (right-configuration park)))

  (pr2-proj-reasoning:check-navigating-collisions ?navigation-location)
  (setf ?navigation-location (desig:current-desig ?navigation-location))

  (cpl:with-failure-handling
      ((common-fail:navigation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans navigate)
                           "Low-level navigation failed: ~a~%.Ignoring anyway." e)
         (return)))
    (exe:perform (desig:an action
                           (type going)
                           (target ?navigation-location)))))


(cpl:def-cram-function turn-towards (?look-target ?robot-location)
  (cpl:with-failure-handling
      ((desig:designator-error (e)
         (roslisp:ros-warn (fd-plans turn-towards)
                           "Desig ~a could not be resolved: ~a~%Cannot look."
                           ?look-target e)
         (cpl:fail 'common-fail:looking-high-level-failure))

       (common-fail:navigation-high-level-failure (e)
         (roslisp:ros-warn (fd-plans turn-towards)
                           "When turning around navigation failure happened: ~a~%~
                              Cannot look."
                           e)
         (cpl:fail 'common-fail:looking-high-level-failure)))

    (cpl:with-retry-counters ((turn-around-retries 1))
      (cpl:with-failure-handling
          ((common-fail:ptu-low-level-failure (e)
             (roslisp:ros-warn (pp-plans turn-towards) "~a~%Turning around." e)
             (cpl:do-retry turn-around-retries
               (cpl:par
                 (exe:perform (desig:an action
                                        (type navigating)
                                        (location ?robot-location)))
                 (exe:perform (desig:an action
                                        (type looking)
                                        (direction forward))))
               (cpl:retry))
             (roslisp:ros-warn (pp-plans turn-towards) "Turning around didn't work :'(~%")
             (cpl:fail 'common-fail:looking-high-level-failure)))

        (exe:perform (desig:an action
                               (type looking)
                               (target ?look-target)))))))


(cpl:def-cram-function manipulate-environment (action-type ?object-to-manipulate
                                                           ?arm ?distance
                                                           ?manipulate-robot-location)
  (cpl:with-failure-handling
      ((desig:designator-error (e)
         (roslisp:ros-warn (fd-plans environment) "~a~%Propagating up." e)
         (cpl:fail 'common-fail:environment-manipulation-impossible
                   :description "Some designator could not be resolved.")))

    (cpl:with-retry-counters ((relocation-retries 10))
      (cpl:with-failure-handling
          (((or common-fail:navigation-goal-in-collision
                common-fail:environment-unreachable
                common-fail:gripper-low-level-failure
                common-fail:manipulation-low-level-failure) (e)
             (roslisp:ros-warn (fd-plans environment) "~a" e)
             (cpl:do-retry relocation-retries
               (setf ?manipulate-robot-location (desig:next-solution ?manipulate-robot-location))
               (if ?manipulate-robot-location
                   (progn
                     (roslisp:ros-info (fd-plans environment) "Relocating...")
                     (cpl:retry))
                   (progn
                     (roslisp:ros-warn (fd-plans environment) "No more samples to try :'(")
                     (cpl:fail 'common-fail:environment-manipulation-impossible
                               :description "No more samples in navigation designator."))))
             (roslisp:ros-warn (fd-plans environment) "No more retries left :'(")
             (cpl:fail 'common-fail:environment-manipulation-impossible
                       :description "No more retries left.")))

        ;; navigate, open / close
        (exe:perform (desig:an action
                               (type navigating)
                               (location ?manipulate-robot-location)))

        (let ((manipulation-action
                (ecase action-type
                  (:accessing (desig:an action
                                        (type opening)
                                        (arm ?arm)
                                        (object ?object-to-manipulate)
                                        (distance ?distance)))
                  (:sealing (desig:an action
                                      (type closing)
                                      (arm ?arm)
                                      (object ?object-to-manipulate)
                                      (distance ?distance))))))

          (pr2-proj-reasoning:check-environment-manipulation-collisions manipulation-action)
          (setf manipulation-action (desig:current-desig manipulation-action))

          (exe:perform manipulation-action))))))


(cpl:def-cram-function search-for-object (?object-designator ?search-location ?robot-location
                                                             &optional (retries 900))
  "Searches for `?object-designator' in its likely location `?search-location'."

  (cpl:with-failure-handling
      ((desig:designator-error (e)
         (roslisp:ros-warn (fd-plans search-for-object)
                           "Desig ~a could not be resolved: ~a~%Propagating up."
                           ?search-location e)
         (cpl:fail 'common-fail:object-nowhere-to-be-found
                   :description "Search location designator could not be resolved.")))
    ;; TODO: IF ROBOT-LOCATION DESIGNATOR CANNOT BE RESOLVED, PICK ANOTHER SEARCH-LOCATION

    ;; take new `?search-location' sample if a failure happens and retry
    (cpl:with-retry-counters ((search-location-retries retries))
      (cpl:with-failure-handling
          (((or common-fail:navigation-goal-in-collision
                common-fail:looking-high-level-failure
                common-fail:perception-low-level-failure) (e)
             (roslisp:ros-warn (fd-plans search-for-object) "~a" e)
             (cpl:do-retry search-location-retries
               (setf ?search-location (desig:next-solution ?search-location))
               (desig:reset ?robot-location)
               (setf ?robot-location (desig:next-solution ?robot-location))
               (if ?search-location
                   (progn
                     (roslisp:ros-warn (fd-plans search-for-object) "Retrying...~%")
                     (cpl:retry))
                   (progn
                     (roslisp:ros-warn (fd-plans search-for-object) "No samples left :'(~%")
                     (cpl:fail 'common-fail:object-nowhere-to-be-found))))
             (roslisp:ros-warn (fd-plans search-for-object) "No retries left :'(~%")
             (cpl:fail 'common-fail:object-nowhere-to-be-found)))

        ;; navigate, look and detect
        (exe:perform (desig:an action
                               (type navigating)
                               (location ?robot-location)))
        (exe:perform (desig:an action
                               (type turning-towards)
                               (target ?search-location)))
        (exe:perform (desig:an action
                               (type detecting)
                               (object ?object-designator)))))))



(cpl:def-cram-function fetch (?object-designator ?arm ?grasp
                                                 ?pick-up-robot-location pick-up-action)
  "Fetches a perceived object `?object-designator' with arm `?arm' (if not NIL)
while standing at `?pick-up-robot-location' (if not NIL)
and using the grasp and arm specified in `pick-up-action' (if not NIL)."
  (cpl:with-failure-handling
      ((desig:designator-error (e)
         (roslisp:ros-warn (fd-plans fetch) "~a~%Propagating up." e)
         (cpl:fail 'common-fail:object-unfetchable
                   :object ?object-designator
                   :description "Some designator could not be resolved.")))

    ;; take a new `?pick-up-robot-location' sample if a failure happens
    (cpl:with-retry-counters ((relocation-for-ik-retries 10))
      (cpl:with-failure-handling
          (((or common-fail:navigation-goal-in-collision
                common-fail:looking-high-level-failure
                common-fail:perception-low-level-failure
                common-fail:object-unreachable
                common-fail:manipulation-low-level-failure) (e)
             (roslisp:ros-warn (fd-plans fetch)
                               "Object of type ~a is unreachable: ~a"
                               (desig:desig-prop-value ?object-designator :type) e)
             (cpl:do-retry relocation-for-ik-retries
               (setf ?pick-up-robot-location (desig:next-solution ?pick-up-robot-location))
               (if ?pick-up-robot-location
                   (progn
                     (roslisp:ros-info (fd-plans fetch) "Relocating...")
                     (cpl:retry))
                   (progn
                     (roslisp:ros-warn (fd-plans fetch) "No more samples to try :'(")
                     (cpl:fail 'common-fail:object-unfetchable))))
             (roslisp:ros-warn (fd-plans fetch) "No more retries left :'(")
             (cpl:fail 'common-fail:object-unfetchable :object ?object-designator)))

        ;; navigate, look, detect and pick-up
        (exe:perform (desig:an action
                               (type navigating)
                               (location ?pick-up-robot-location)))
        (exe:perform (desig:an action
                               (type turning-towards)
                               (target (desig:a location (of ?object-designator)))))

        (cpl:with-retry-counters ((regrasping-retries 1))
          (cpl:with-failure-handling
              ((common-fail:gripper-low-level-failure (e)
                 (roslisp:ros-warn (fd-plans fetch) "Misgrasp happened: ~a~%" e)
                 (cpl:do-retry regrasping-retries
                   (roslisp:ros-info (fd-plans fetch) "Reperceiving and repicking...")
                   (exe:perform (desig:an action
                                          (type positioning-arm)
                                          (left-configuration park)
                                          (right-configuration park)))
                   (cpl:retry))
                 (roslisp:ros-warn (fd-plans fetch) "No more regrasping retries left :'(")
                 (cpl:fail 'common-fail:object-unreachable
                           :description "Misgrasp happened and retrying didn't help.")))

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
                                                 _left-grasp-poses _right-grasp-poses
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
                                    (desig:when ?grasp
                                      (grasp ?grasp))
                                    (object ?more-precise-perceived-object-desig)))))

                (setf pick-up-action (desig:current-desig pick-up-action))
                (pr2-proj-reasoning:check-picking-up-collisions pick-up-action)
                (setf pick-up-action (desig:current-desig pick-up-action))

                (exe:perform pick-up-action)))))))

    (exe:perform (desig:an action
                           (type positioning-arm)
                           (left-configuration park)
                           (right-configuration park)))
    (desig:current-desig ?object-designator)))


(cpl:def-cram-function deliver (?object-designator ?arm ?target-location
                                                   ?target-robot-location place-action)
  ;; Reference the `?target-location' to see if that works at all
  ;; If not, delivering is impossible so throw a OBJECT-UNDERLIVERABLE failure
  (cpl:with-failure-handling
      ((desig:designator-error (e)
         (roslisp:ros-warn (fd-plans deliver) "~a~%Propagating up." e)
         (cpl:fail 'common-fail:object-undeliverable
                   :description "Some designator could not be resolved.")))

    (cpl:with-retry-counters ((outer-target-location-retries 2))
      (cpl:with-failure-handling
          (((or desig:designator-error
                common-fail:object-undeliverable) (e)
             (roslisp:ros-warn (fd-plans deliver)
                               "Undeliverable. Last chance -- another target location.~%~a" e)
             (cpl:do-retry outer-target-location-retries
               (let ((next-target-location (desig:next-solution ?target-location)))
                 (if next-target-location
                     (progn
                       (roslisp:ros-info (fd-plans deliver) "Retrying with new placement...")
                       (setf ?target-location next-target-location)
                       (desig:reset ?target-robot-location)
                       (cpl:retry))
                     (progn
                       (roslisp:ros-warn (fd-plans deliver) "No more placement samples :'(")
                       (cpl:fail 'common-fail:object-undeliverable)))))
             (roslisp:ros-warn (fd-plans deliver) "No more re-placement retries left :'(")
             (cpl:fail 'common-fail:object-undeliverable)))

        ;; take a new `?target-robot-location' sample if a failure happens
        (cpl:with-retry-counters ((relocation-for-ik-retries 4))
          (cpl:with-failure-handling
              (((or common-fail:navigation-goal-in-collision
                    common-fail:object-undeliverable
                    common-fail:manipulation-low-level-failure) (e)
                 (roslisp:ros-warn (fd-plans deliver)
                                   "Object is undeliverable from current base location.~%~a" e)
                 (cpl:do-retry relocation-for-ik-retries
                   (setf ?target-robot-location
                         (desig:next-solution ?target-robot-location))
                   (if ?target-robot-location
                       (progn
                         (roslisp:ros-info (fd-plans deliver) "Relocating...")
                         (cpl:retry))
                       (progn
                         (roslisp:ros-warn (fd-plans deliver) "No more relocation samples :'(")
                         (cpl:fail 'common-fail:object-undeliverable))))
                 (roslisp:ros-warn (fd-plans deliver) "No more relocation retries left :'(")
                 (cpl:fail 'common-fail:object-undeliverable)))

            ;; navigate
            (exe:perform (desig:an action
                                   (type navigating)
                                   (location ?target-robot-location)))

            ;; take a new `?target-location' sample if a failure happens
            (cpl:with-retry-counters ((target-location-retries 3))
              (cpl:with-failure-handling
                  (((or common-fail:looking-high-level-failure
                        common-fail:object-unreachable) (e)
                     (roslisp:ros-warn (fd-plans deliver) "Placing failed: ~a" e)
                     (cpl:do-retry target-location-retries
                       (let ((next-target-location (desig:next-solution ?target-location)))
                         (if next-target-location
                             (progn
                               (roslisp:ros-warn (fd-plans deliver)
                                                 "Retrying with new placing location...~%")
                               (setf ?target-location next-target-location)
                               (desig:reset ?target-robot-location)
                               (cpl:retry))
                             (progn
                               (roslisp:ros-warn (fd-plans deliver)
                                                 "No target location samples left :'(~%")
                               (cpl:fail 'common-fail:object-undeliverable)))))
                     (roslisp:ros-warn (fd-plans deliver)
                                       "No target-location-retries left :'(~%")
                     (cpl:fail 'common-fail:object-undeliverable)))

                ;; look
                (exe:perform (desig:an action
                                       (type turning-towards)
                                       (target ?target-location)))

                ;; place
                (let ((place-action
                        (or (when place-action
                              (destructuring-bind (_action _object-designator _on-obj-desig
                                                   _assemblage-name
                                                   ?arm _gripper-opening
                                                   _left-reach-poses _right-reach-poses
                                                   _left-put-poses _right-put-poses
                                                   _left-lift-poses _right-lift-poses
                                                   ?projected-target-location)
                                  (desig:reference place-action)
                                (desig:an action
                                          (type placing)
                                          (arm ?arm)
                                          (object ?object-designator)
                                          (target ?projected-target-location))))
                            (desig:an action
                                      (type placing)
                                      (desig:when ?arm
                                        (arm ?arm))
                                      (object ?object-designator)
                                      (target ?target-location)))))

                  (setf place-action (desig:current-desig place-action))
                  (pr2-proj-reasoning:check-placing-collisions place-action)
                  (setf place-action (desig:current-desig place-action))

                  (exe:perform place-action))))))))))



(defun drop-at-sink ()
  (let ((?base-pose-in-map
          ;; (cl-transforms-stamped:make-pose-stamped
          ;;  cram-tf:*fixed-frame*
          ;;  0.0
          ;;  (cl-transforms:make-3d-vector 0.7 -0.2 0)
          ;;  (cl-transforms:make-identity-rotation))
          (cl-transforms-stamped:make-pose-stamped
           cram-tf:*fixed-frame*
           0.0
           (cl-transforms:make-3d-vector 0 0 0)
           (cl-transforms:make-quaternion 0 0 -1 1)))
        ;; (?placing-pose
        ;;   (cl-transforms-stamped:make-pose-stamped
        ;;    cram-tf:*robot-base-frame*
        ;;    0.0
        ;;    (cl-transforms:make-3d-vector 0.7 0 1.2)
        ;;    (cl-transforms:make-identity-rotation)))
        )
    (cpl:with-failure-handling
        ((common-fail:navigation-low-level-failure (e)
           (declare (ignore e))
           (return)))
      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location (pose ?base-pose-in-map)))))))
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (declare (ignore e))
         (return)))
    (exe:perform
     (desig:an action
               (type placing)
               ;; (target (desig:a location
               ;;                  (pose ?placing-pose)))
               ))))


(cpl:def-cram-function transport (?object-designator ?search-location ?delivering-location
                                                     ?arm ?grasp
                                                     search-location-accessible)
  (unless search-location-accessible
    (exe:perform (desig:an action
                           (type accessing)
                           (location ?search-location)
                           (distance 0.3))))

  (unwind-protect
       (let ((?perceived-object-designator
               (exe:perform (desig:an action
                                      (type searching)
                                      (object ?object-designator)
                                      (location ?search-location)))))
         (roslisp:ros-info (pp-plans transport)
                           "Found object of type ~a."
                           (desig:desig-prop-value ?perceived-object-designator :type))

         (let ((?fetch-robot-location
                 (desig:a location
                          (reachable-for pr2)
                          (when ?arm
                            (arm ?arm))
                          (object ?object-designator)))
               ?fetch-pick-up-action
               (?deliver-robot-location
                 (desig:a location
                          (reachable-for pr2)
                          (location ?delivering-location)))
               ?deliver-place-action)

           ;; If running on the real robot, execute below task tree in projection
           ;; N times first, then pick the best parameterization
           ;; and use that parameterization in the real world.
           ;; If running in projection, just execute the task tree below as normal.
           (pr2-proj-reasoning:with-projected-task-tree
               (?fetch-robot-location ?fetch-pick-up-action
                                      ?deliver-robot-location ?deliver-place-action)
               3
               #'pr2-proj-reasoning:pick-best-parameters-by-distance

             (let ((?fetched-object
                     (exe:perform (desig:an action
                                            (type fetching)
                                            (desig:when ?arm
                                              (arm ?arm))
                                            (desig:when ?grasp
                                              (grasp ?grasp))
                                            (object ?perceived-object-designator)
                                            (robot-location ?fetch-robot-location)
                                            (pick-up-action ?fetch-pick-up-action)))))

               (roslisp:ros-info (pp-plans transport) "Fetched the object.")
               (cpl:with-failure-handling
                   ((common-fail:object-undeliverable (e)
                      (declare (ignore e))
                      (drop-at-sink)
                      ;; (return)
                      ))
                 (exe:perform (desig:an action
                                        (type delivering)
                                        (when ?arm
                                          (arm ?arm))
                                        (object ?fetched-object)
                                        (target ?delivering-location)
                                        (robot-location ?deliver-robot-location)
                                        (place-action ?deliver-place-action))))))))

    (unless search-location-accessible
      (exe:perform (desig:an action
                             (type sealing)
                             (location ?search-location)
                             (distance 0.3))))))
