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

(in-package :fd-plans)

(defun go-without-collisions (&key
                                ((:location ?navigation-location))
                              &allow-other-keys)
  (declare (type desig:location-designator ?navigation-location))
  "Check if navigation goal is in reach, if not propagate failure up,
if yes, perform GOING action while ignoring failures."

  (exe:perform (desig:an action
                         (type positioning-arm)
                         (left-configuration park)
                         (right-configuration park)))

  (proj-reasoning:check-navigating-collisions ?navigation-location)
  (setf ?navigation-location (desig:current-desig ?navigation-location))

  (cpl:with-failure-handling
      ((common-fail:navigation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans navigate)
                           "Low-level navigation failed: ~a~%.Ignoring anyway." e)
         (return)))
    (exe:perform (desig:an action
                           (type going)
                           (target ?navigation-location)))))


(defun turn-towards (&key
                       ((:target ?look-target))
                       ((:robot-location ?robot-location))
                     &allow-other-keys)
  (declare (type desig:location-designator ?look-target ?robot-location))
  "Perform a LOOKING action, if looking target twists the neck,
turn the robot base such that it looks in the direction of target and look again."
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


(defun manipulate-environment (&key
                                 ((:type action-type))
                                 ((:object ?object-to-manipulate))
                                 ((:arm ?arm))
                                 ((:distance ?distance))
                                 ((:robot-location ?manipulate-robot-location))
                               &allow-other-keys)
  (declare (type keyword action-type ?arm)
           (type desig:object-designator ?object-to-manipulate)
           (type (or number null) ?distance)
           ;; here, ?manipulate-robot-location can only be null within the function
           ;; but one should not pass a NULL location as argument,
           ;; otherwise it will just cpl:fail straight away.
           (type (or null desig:location-designator) ?manipulate-robot-location))
  "Navigate to reachable location, check if opening/closing trajectory causes collisions,
if yes, relocate and retry, if no collisions, open or close container."

  (cpl:with-failure-handling
      ((desig:designator-error (e)
         (roslisp:ros-warn (fd-plans environment) "~a~%Propagating up." e)
         (cpl:fail 'common-fail:environment-manipulation-impossible
                   :description "Some designator could not be resolved.")))

    (cpl:with-retry-counters ((relocation-retries 50))
      (cpl:with-failure-handling
          (((or common-fail:navigation-goal-in-collision
                common-fail:environment-unreachable
                common-fail:gripper-low-level-failure
                common-fail:manipulation-low-level-failure) (e)
             (common-fail:retry-with-loc-designator-solutions
                 ?manipulate-robot-location
                 relocation-retries
                 (:error-object-or-string e
                  :warning-namespace (fd-plans environment)
                  :rethrow-failure 'common-fail:environment-manipulation-impossible)
               (roslisp:ros-info (fd-plans environment) "Relocating..."))))

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
                                        (desig:when ?distance
                                          (distance ?distance))))
                  (:sealing (desig:an action
                                      (type closing)
                                      (arm ?arm)
                                      (object ?object-to-manipulate)
                                      (desig:when ?distance
                                        (distance ?distance)))))))

          (proj-reasoning:check-environment-manipulation-collisions manipulation-action)
          (setf manipulation-action (desig:current-desig manipulation-action))

          (exe:perform manipulation-action))))))


(defun search-for-object (&key
                            ((:object ?object-designator))
                            ((:location ?search-location))
                            ((:robot-location ?robot-location))
                          &allow-other-keys)
  (declare (type desig:object-designator ?object-designator)
           ;; location desigs can turn NILL in the course of execution
           ;; but should not be passed as NILL to start with.
           (type (or desig:location-designator null) ?search-location ?robot-location))
  "Searches for `?object-designator' in its likely location `?search-location'.
If the object is not there or navigation location is unreachable,
retries with different search location or robot base location."

  (format t "~%~% ?search-location in SEARCH plan ~a ~%" ?search-location)
  (sleep 5);;TODO REMOVE
  (cpl:with-failure-handling
      ((desig:designator-error (e)
         (roslisp:ros-warn (fd-plans search-for-object)
                           "Desig ~a could not be resolved: ~a~%Propagating up."
                           ?search-location e)
         (cpl:fail 'common-fail:object-nowhere-to-be-found
                   :description "Search location designator could not be resolved.")))

    ;; take new `?search-location' sample if a failure happens and retry
    (cpl:with-retry-counters ((outer-search-location-retries 10))
      (cpl:with-failure-handling
          ((common-fail:object-nowhere-to-be-found (e)
             (common-fail:retry-with-loc-designator-solutions
                 ?search-location
                 outer-search-location-retries
                 (:error-object-or-string e
                  :warning-namespace (fd-plans search-for-object)
                  :reset-designators (list ?robot-location)
                  :rethrow-failure 'common-fail:object-nowhere-to-be-found)
               (roslisp:ros-warn (fd-plans search-for-object)
                                 "Search is about to give up. Retrying~%"))))

        ;; if the going action fails, pick another `?robot-location' sample and retry
        (cpl:with-retry-counters ((robot-location-retries 3))
          (cpl:with-failure-handling
              (((or common-fail:navigation-goal-in-collision
                    common-fail:looking-high-level-failure
                    common-fail:perception-low-level-failure) (e)
                 (common-fail:retry-with-loc-designator-solutions
                     ?robot-location
                     robot-location-retries
                     (:error-object-or-string e
                      :warning-namespace (fd-plans search-for-object)
                      :reset-designators (list ?search-location)
                      :rethrow-failure 'common-fail:object-nowhere-to-be-found))))

            ;; navigate
            (exe:perform (desig:an action
                                   (type navigating)
                                   (location ?robot-location)))

            ;; if perception action fails, try another `?search-location' and retry
            (cpl:with-retry-counters ((search-location-retries 3))
              (cpl:with-failure-handling
                  (((or common-fail:perception-low-level-failure
                        common-fail:looking-high-level-failure) (e)
                     (common-fail:retry-with-loc-designator-solutions
                         ?search-location
                         search-location-retries
                         (:error-object-or-string e
                          :warning-namespace (fd-plans search-for-object)
                          :reset-designators (list ?robot-location)))))

                (exe:perform (desig:an action
                                       (type turning-towards)
                                       (target ?search-location)))
                (exe:perform (desig:an action
                                       (type detecting)
                                       (object ?object-designator)))))))))))



(defun fetch (&key
                ((:object ?object-designator))
                ((:arms ?arms))
                ((:grasps ?grasps))
                ((:robot-location ?pick-up-robot-location))
                pick-up-action
              &allow-other-keys)
  (declare (type desig:object-designator ?object-designator)
           (type list ?arms ?grasps)
           ;; ?pick-up-robot-location should not be NULL at the beginning
           ;; but can become NULL during execution of the plan
           (type (or desig:location-designator null) ?pick-up-robot-location)
           (type (or desig:action-designator null) pick-up-action))
  "Fetches a perceived object `?object-designator' with
one of arms in the `?arms' lazy list (if not NIL) and one of grasps in `?grasps' if not NIL,
while standing at `?pick-up-robot-location'
and using the grasp and arm specified in `pick-up-action' (if not NIL)."

  (cpl:with-failure-handling
      ((desig:designator-error (e)
         (roslisp:ros-warn (fd-plans fetch) "~a~%Propagating up." e)
         (cpl:fail 'common-fail:object-unfetchable
                   :object ?object-designator
                   :description "Some designator could not be resolved.")))

    ;; take a new `?pick-up-robot-location' sample if a failure happens
    (cpl:with-retry-counters ((relocation-for-ik-retries 20))
      (cpl:with-failure-handling
          (((or common-fail:navigation-goal-in-collision
                common-fail:looking-high-level-failure
                common-fail:perception-low-level-failure
                common-fail:object-unreachable
                common-fail:manipulation-low-level-failure) (e)
             (common-fail:retry-with-loc-designator-solutions
                 ?pick-up-robot-location
                 relocation-for-ik-retries
                 (:error-object-or-string
                  (format NIL "Object of type ~a is unreachable: ~a"
                          (desig:desig-prop-value ?object-designator :type) e)
                  :warning-namespace (fd-plans fetch)
                  :rethrow-failure 'common-fail:object-unfetchable))))

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


              (let ((?arm (cut:lazy-car ?arms)))
                ;; if picking up fails, try another arm
                (cpl:with-retry-counters ((arm-retries 1))
                  (cpl:with-failure-handling
                      (((or common-fail:manipulation-low-level-failure
                            common-fail:object-unreachable
                            desig:designator-error) (e)
                         (common-fail:retry-with-list-solutions
                             ?arms
                             arm-retries
                             (:error-object-or-string
                              (format NIL "Manipulation failed: ~a.~%Next." e)
                              :warning-namespace (kvr plans)
                              :rethrow-failure 'common-fail:object-unreachable)
                           (setf ?arm (cut:lazy-car ?arms)))))

                    (let ((?grasp (cut:lazy-car ?grasps)))
                      ;; if picking up fails, try another grasp orientation
                      (cpl:with-retry-counters ((grasp-retries 4))
                        (cpl:with-failure-handling
                            (((or common-fail:manipulation-low-level-failure
                                  common-fail:object-unreachable
                                  desig:designator-error) (e)
                               (common-fail:retry-with-list-solutions
                                   ?grasps
                                   grasp-retries
                                   (:error-object-or-string
                                    (format NIL "Picking up failed: ~a.~%Next" e)
                                    :warning-namespace (kvr plans))
                                 (setf ?grasp (cut:lazy-car ?grasps)))))

                          (let ((pick-up-action
                                  ;; if pick-up-action already exists,
                                  ;; use its params for picking up
                                  (or (when pick-up-action
                                        (let* ((referenced-action-desig
                                                 (desig:reference pick-up-action))
                                               (?arm
                                                 (desig:desig-prop-value
                                                  referenced-action-desig
                                                  :arm))
                                               (?grasp
                                                 (desig:desig-prop-value
                                                  referenced-action-desig
                                                  :grasp)))
                                          (desig:an action
                                                    (type picking-up)
                                                    (arm ?arm)
                                                    (grasp ?grasp)
                                                    (object
                                                     ?more-precise-perceived-object-desig))))
                                      (desig:an action
                                                (type picking-up)
                                                (desig:when ?arm
                                                  (arm ?arm))
                                                (desig:when ?grasp
                                                  (grasp ?grasp))
                                                (object
                                                 ?more-precise-perceived-object-desig)))))

                            (setf pick-up-action (desig:current-desig pick-up-action))
                            (proj-reasoning:check-picking-up-collisions pick-up-action)
                            (setf pick-up-action (desig:current-desig pick-up-action))

                            (exe:perform pick-up-action)

                            (exe:perform (desig:an action
                                                   (type positioning-arm)
                                                   (left-configuration park)
                                                   (right-configuration park)))
                            (desig:current-desig ?object-designator)))))))))))))))





(defun deliver (&key
                  ((:object ?object-designator))
                  ((:arm ?arm))
                  ((:target ?target-location))
                  ((:robot-location ?target-robot-location))
                  place-action
                &allow-other-keys)
  (declare (type desig:object-designator ?object-designator)
           (type (or keyword null) ?arm)
           ;; don't pass NULL as ?target-location or ?target-robot-location!
           ;; they can turn NULL during execution but not at the beginning
           (type (or desig:location-designator null) ?target-location ?target-robot-location)
           (type (or desig:action-designator null) place-action))
  "Delivers `?object-designator' to `?target-location', where object is held in `?arm'
and the robot should stand at `?target-robot-location' when placing the object.
If a failure happens, try a different `?target-location' or `?target-robot-location'."

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
             (common-fail:retry-with-loc-designator-solutions
                 ?target-location
                 outer-target-location-retries
                 (:error-object-or-string
                  (format NIL "Undeliverable. Trying another target location.~%~a" e)
                  :warning-namespace (fd-plans deliver)
                  :reset-designators (list ?target-robot-location)
                  :rethrow-failure 'common-fail:object-undeliverable))))

        ;; take a new `?target-robot-location' sample if a failure happens
        (cpl:with-retry-counters ((relocation-for-ik-retries 4))
          (cpl:with-failure-handling
              (((or common-fail:navigation-goal-in-collision
                    common-fail:object-undeliverable
                    common-fail:manipulation-low-level-failure) (e)
                 (common-fail:retry-with-loc-designator-solutions
                     ?target-robot-location
                     relocation-for-ik-retries
                     (:error-object-or-string
                      (format NIL "Object is undeliverable from base location.~%~a" e)
                      :warning-namespace (fd-plans deliver)
                      :rethrow-failure 'common-fail:object-undeliverable))))

            ;; navigate
            (exe:perform (desig:an action
                                   (type navigating)
                                   (location ?target-robot-location)))

            ;; take a new `?target-location' sample if a failure happens
            (cpl:with-retry-counters ((target-location-retries 9))
              (cpl:with-failure-handling
                  (((or common-fail:looking-high-level-failure
                        common-fail:object-unreachable
                        common-fail:high-level-failure) (e)
                     (common-fail:retry-with-loc-designator-solutions
                         ?target-location
                         target-location-retries
                         (:error-object-or-string (format NIL "Placing failed: ~a" e)
                          :warning-namespace (fd-plans deliver)
                          :reset-designators (list ?target-robot-location)
                          :rethrow-failure 'common-fail:object-undeliverable)
                       (roslisp:ros-warn (fd-plans deliver)
                                         "Retrying with new placing location ...~%"))))

                ;; look
                (exe:perform (desig:an action
                                       (type turning-towards)
                                       (target ?target-location)))

                ;; place
                (let ((place-action
                        (or (when place-action
                              (let* ((referenced-action-desig
                                       (desig:reference place-action))
                                     (?arm
                                       (desig:desig-prop-value referenced-action-desig :arm))
                                     (?projected-target-location
                                       (desig:desig-prop-value referenced-action-desig :target)))
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

                  ;; test if the placing trajectory is reachable and not colliding
                  (setf place-action (desig:current-desig place-action))
                  (proj-reasoning:check-placing-collisions place-action)
                  (setf place-action (desig:current-desig place-action))

                  ;; test if the placing pose is a good one -- not falling on the floor
                  ;; test function throws a high-level-failure if not good pose
                  (proj-reasoning:check-placing-pose-stability
                   ?object-designator ?target-location)

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


(defun transport (&key
                    ((:object ?object-designator))
                    ((:search-location ?search-location))
                    ((:search-robot-location ?search-base-location))
                    ((:fetch-robot-location ?fetch-robot-location))
                    ((:arm ?arm))
                    ((:grasp ?grasp))
                    ((:arms ?arms))
                    ((:grasps ?grasps))
                    ((:deliver-location ?delivering-location))
                    ((:deliver-robot-location ?deliver-robot-location))
                    search-location-accessible
                    delivery-location-accessible
                  &allow-other-keys)
  (format t "~% TRANSPORT PLAN ~% ?search-pose: ~a ~%" ?search-location)
  (sleep 5)
  (unless search-location-accessible
    (exe:perform (desig:an action
                           (type accessing)
                           (location ?search-location)
                           (distance 0.48))))

  (unwind-protect
       (let ((?perceived-object-designator
               (exe:perform (desig:an action
                                      (type searching)
                                      (object ?object-designator)
                                      (location ?search-location)
                                      (desig:when ?search-base-location
                                        (robot-location ?search-base-location)))))
             ;; (?robot-name
             ;;   (cut:var-value '?robot-name
             ;;                  (car (prolog:prolog '(rob-int:robot ?robot-name)))))
             )
         (roslisp:ros-info (pp-plans transport)
                           "Found object of type ~a."
                           (desig:desig-prop-value ?perceived-object-designator :type))

         ;; (unless ?fetch-robot-location
         ;;   (setf ?fetch-robot-location
         ;;         (desig:a location
         ;;                  (reachable-for ?robot-name)
         ;;                  (desig:when ?arm
         ;;                    (arm ?arm))
         ;;                  (object ?perceived-object-designator))))
         ;; (unless ?deliver-robot-location
         ;;   (setf ?deliver-robot-location
         ;;         (desig:a location
         ;;                  (reachable-for ?robot-name)
         ;;                  (location ?delivering-location))))

         ;; If running on the real robot, execute below task tree in projection
         ;; N times first, then pick the best parameterization
         ;; and use that parameterization in the real world.
         ;; If running in projection, just execute the task tree below as normal.
         (let (?fetch-pick-up-action ?deliver-place-action)
           (proj-reasoning:with-projected-task-tree
               (?fetch-robot-location ?fetch-pick-up-action
                                      ?deliver-robot-location ?deliver-place-action)
               3
               #'proj-reasoning:pick-best-parameters-by-distance

             (let ((?fetched-object
                     (exe:perform (desig:an action
                                            (type fetching)
                                            (desig:when ?arm
                                              (arm ?arm))
                                            (desig:when ?grasp
                                              (grasp ?grasp))
                                            (desig:when ?arms
                                              (arms ?arms))
                                            (desig:when ?grasps
                                              (grasps ?grasps))
                                            (object ?perceived-object-designator)
                                            (desig:when ?fetch-robot-location
                                              (robot-location ?fetch-robot-location))
                                            (pick-up-action ?fetch-pick-up-action)))))

               (roslisp:ros-info (pp-plans transport) "Fetched the object.")
               (cpl:with-failure-handling
                   ((common-fail:object-undeliverable (e)
                      (declare (ignore e))
                      (drop-at-sink)
                      ;; (return)
                      ))
                 (unless delivery-location-accessible
                   (exe:perform (desig:an action
                                          (type accessing)
                                          (location ?delivering-location)
                                          (distance 0.3))))
                 (unwind-protect
                      (exe:perform (desig:an action
                                             (type delivering)
                                             (desig:when ?arm
                                               (arm ?arm))
                                             (object ?fetched-object)
                                             (target ?delivering-location)
                                             (desig:when ?deliver-robot-location
                                               (robot-location ?deliver-robot-location))
                                             (place-action ?deliver-place-action)))
                   (unless delivery-location-accessible
                     (exe:perform (desig:an action
                                            (type sealing)
                                            (location ?delivering-location)
                                            (distance 0.3))))))))))

    (unless search-location-accessible
      (exe:perform (desig:an action
                             (type sealing)
                             (location ?search-location)
                             (distance 0.48))))))
