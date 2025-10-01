;;;
;;; Copyright (c) 2025, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :demos)

(defun new-pour (&key
                   ((:source-object ?source-object))
                   ((:context ?context))

                   ;; ((:access-search-robot-location ?access-search-robot-location))
                   ;; ((:seal-search-robot-location ?seal-search-robot-location))
                   ;; ((:access-seal-search-arms ?access-seal-search-arms))
                   ;; ((:access-seal-search-grasps ?access-seal-search-grasps))
                   ;; ((:access-search-outer-robot-location ?access-search-outer-robot-location))
                   ;; ((:seal-search-outer-robot-location ?seal-search-outer-robot-location))
                   ;; ((:access-seal-search-outer-arms ?access-seal-search-outer-arms))
                   ;; ((:access-search-outer-grasps ?access-search-outer-grasps))
                   ;; ((:seal-search-outer-grasps ?seal-search-outer-grasps))

                   ;; ((:access-deliver-robot-location ?access-deliver-robot-location))
                   ;; ((:seal-deliver-robot-location ?seal-deliver-robot-location))
                   ;; ((:access-seal-deliver-arms ?access-seal-deliver-arms))
                   ;; ((:access-seal-deliver-grasps ?access-seal-deliver-grasps))
                   ;; ((:access-deliver-outer-robot-location ?access-deliver-outer-robot-location))
                   ;; ((:seal-deliver-outer-robot-location ?seal-deliver-outer-robot-location))
                   ;; ((:access-seal-deliver-outer-arms ?access-seal-deliver-outer-arms))
                   ;; ((:access-seal-deliver-outer-grasps ?access-seal-deliver-outer-grasps))

                   ;; ((:search-robot-location ?search-robot-location))
                   ;; ((:fetch-robot-location ?fetch-robot-location))
                   ((:arms ?arms))
                   ((:sides ?sides))

                   ((:target-object ?target-object))
                   ((:target-search-location ?target-search-location))
                   ;; ((:deliver-robot-location ?deliver-robot-location))
                 &allow-other-keys)

  (let ((?arm (first ?arms)))
    (let (;; (?gripper-link (if (eq ?arm :left)
          ;;                    "l_wrist_roll_link"
          ;;                    "r_wrist_roll_link"))
          (?goal `(cpoe:object-in-hand ,?source-object ,?arm)))
      (exe:perform (desig:an action
                             (type transporting)
                             (object ?source-object)
                             (arms (?arm))
                             (context ?context)
                             (target (a location
                                        (IN (An OBJECT
                                                (TYPE ROBOT)
                                                (NAME PR2)
                                                (PART-OF PR2)
                                                ;; (URDF-NAME ?gripper-link)
                                                ))))
                             (goal ?goal))))
    (setf ?source-object (desig:current-desig ?source-object))

    ;; if we are not sure about the exact location of target object find it
    (let ((?goal `(man-int:location-certain ,?target-search-location)))
      (exe:perform (desig:an action
                             (type searching)
                             (location ?target-search-location)
                             (goal ?goal))))
    ;; if deliver-location is inside a container, open the container
    (let ((?goal `(cpoe:location-accessible ,?target-search-location)))
      (exe:perform (desig:an action
                             (type accessing)
                             (location ?target-search-location)
                             ;; (desig:when ?access-deliver-robot-location
                             ;;   (robot-location ?access-deliver-robot-location))
                             ;; (desig:when ?access-seal-deliver-arms
                             ;;   (arms ?access-seal-deliver-arms))
                             ;; (desig:when ?access-seal-deliver-grasps
                             ;;   (grasps ?access-seal-deliver-grasps))
                             ;; (desig:when ?access-deliver-outer-robot-location
                             ;;   (outer-robot-location ?access-deliver-outer-robot-location))
                             ;; (desig:when ?access-seal-deliver-outer-arms
                             ;;   (outer-arms ?access-seal-deliver-outer-arms))
                             ;; (desig:when ?access-seal-deliver-outer-grasps
                             ;;   (outer-grasps ?access-seal-deliver-outer-grasps))
                             (goal ?goal))))

    ;; search for the object to find it's exact pose
    (let ((?goal `(or (cpoe:object-in-hand ,?target-object :left-or-right)
                      (cpoe:object-at-location ,?target-object ,?target-search-location))))
      (exe:perform (desig:an action
                             (type searching)
                             (object ?target-object)
                             (desig:when ?context
                               (context ?context))
                             (goal ?goal))))
    (setf ?target-object (desig:current-desig ?target-object))
    (roslisp:ros-info (pp-plans new-pour)
                      "Found object of type ~a."
                      (desig:desig-prop-value ?target-object :type))

    ;; pour
    (roslisp:ros-info (pp-plans new-pour)
                      "Pouring mobile. Arm: ~a, sides: ~a."
                      ?arm ?sides)
    (new-pour-mobile :source-object (desig:current-desig ?source-object)
                     :target-object (desig:current-desig ?target-object)
                     :arm ?arm
                     :sides ?sides
                     :robot-location (desig:a location
                                              (reachable-for pr2)
                                              (object ?target-object))
                     :look-location (desig:a location
                                             (of ?target-object)))

    (roslisp:ros-info (pp-plans transport) "poured"))

  ;; reset the target location
  (let ((?goal `(cpoe:location-reset ,?target-search-location)))
    (exe:perform (desig:an action
                           (type sealing)
                           (location ?target-search-location)
                           (goal ?goal))))

  (desig:current-desig ?source-object))




(defun calculate-pour-trajectory (source-object
                                  target-object
                                  side
                                  tilt-angle)
  (let* ((grasp (cut:var-value '?grasp
                               (car (prolog:prolog
                                     `(cpoe:object-in-hand
                                       ?object
                                       ?arm
                                       ?grasp)))))
         (arm (cut:var-value '?arm
                             (car (prolog:prolog
                                   `(cpoe:object-in-hand
                                     ?object
                                     ?arm
                                     ?grasp)))))
         (source-object-name
           (desig:desig-prop-value source-object :name))
         (source-object-type
           (desig:desig-prop-value source-object :type))
         (target-object-name
           (desig:desig-prop-value target-object :name))
         (target-object-type
           (desig:desig-prop-value target-object :type))
         (b-T-to
           (man-int:get-object-transform target-object))
         (ros-source-object-name
           (roslisp-utilities:rosify-underscores-lisp-name source-object-name))
         (ros-target-object-name
           (roslisp-utilities:rosify-underscores-lisp-name target-object-name))
         (to-T-so
           (cl-transforms-stamped:make-transform-stamped
            ros-target-object-name
            ros-source-object-name
            0.0
            (case side
              (:front (cl-transforms:make-3d-vector -0.12 0 0.16))
              (:second-front (if (eq grasp :front-flipped)
                                 (cl-transforms:make-3d-vector 0 -0.1 0.2)
                                 (cl-transforms:make-3d-vector 0 0.1 0.2)))
              (:left (if (eq grasp :front-flipped)
                         (cl-transforms:make-3d-vector -0.1 0 0.2)
                         (cl-transforms:make-3d-vector 0.1 0 0.2)))
              (:right (if (eq grasp :front-flipped)
                          (cl-transforms:make-3d-vector 0.1 0 0.2)
                          (cl-transforms:make-3d-vector -0.1 0 0.2)))
              (:back (if (eq grasp :front-flipped)
                         (cl-transforms:make-3d-vector 0 0.1 0.2)
                         (cl-transforms:make-3d-vector 0 -0.1 0.2)))
              (t (error "can only pour from :side or :front")))
            (case side
              (:front (cl-tf:make-identity-rotation))
              (:second-front (cl-transforms:axis-angle->quaternion
                              (cl-transforms:make-3d-vector 0 0 1)
                              (* pi -1/2)))
              (:left (cl-transforms:axis-angle->quaternion
                      (cl-transforms:make-3d-vector 0 0 1)
                      pi))
              (:right (cl-transforms:axis-angle->quaternion
                       (cl-transforms:make-3d-vector 0 0 1)
                       0.0))
              (:back (cl-transforms:axis-angle->quaternion
                        (cl-transforms:make-3d-vector 0 0 1)
                        (* pi 1/2)))
              (t (error "can only pour from :side or :front"))))
           #+for-overloaded-methods-use-translate-pose-and-rotate-pose
           (translate-pose grasp-pose
                           :x-offset (case grasp
                                       (:front (- *pour-xy-offset*))
                                       (:side 0.0)
                                       (error "can only pour from :side or :front"))
                           :y-offset (case grasp
                                       (:front 0.0)
                                       (:side (case arm
                                                (:left *pour-xy-offset*)
                                                (:right (- *pour-xy-offset*))
                                                (t (error "arm can only be :left or :right"))))
                                       (error "can only pour from :side or :front"))
                           :z-offset (+ *bottle-grasp-z-offset*
                                        *pour-z-offset*)))
         (so-T-stdg
           (man-int:get-object-type-to-gripper-transform
            source-object-type source-object-name arm grasp))
         (to-T-stdg
           (reduce #'cram-tf:apply-transform
                   `(,to-T-so ,so-T-stdg)
                   :from-end T))
         (to-T-so-tilts
           (case side
             (:front (cram-tf:rotate-transform-in-own-frame
                      to-T-so :y tilt-angle))
             (:second-front (cram-tf:rotate-transform-in-own-frame
                             to-T-so :y (if (eq grasp :front-flipped)
                                            (- tilt-angle)
                                            tilt-angle)))
             (:left (cram-tf:rotate-transform-in-own-frame
                     to-T-so :x (if (eq grasp :front-flipped)
                                    (- tilt-angle)
                                    tilt-angle)))
             (:right (cram-tf:rotate-transform-in-own-frame
                      to-T-so :x (if (eq grasp :front-flipped)
                                     tilt-angle
                                     (- tilt-angle))))
             (:back (cram-tf:rotate-transform-in-own-frame
                     to-T-so :y (if (eq grasp :front-flipped)
                                    (- tilt-angle)
                                    tilt-angle)))
             (t (error "can only pour from :side or :front"))))
         (to-T-stdg-tilts
           (reduce #'cram-tf:apply-transform
                   `(,to-T-so-tilts ,so-T-stdg)
                   :from-end T)))
    (list (man-int:calculate-gripper-pose-in-map b-T-to arm to-T-stdg)
          (man-int:calculate-gripper-pose-in-map b-T-to arm to-T-stdg-tilts)))
  )


(defun new-pour-mobile (&key
                          ((:source-object ?source-object))
                          ((:target-object ?target-object))
                          ((:arm ?arm))
                          ((:sides ?all-sides))
                          ((:robot-location ?pour-robot-location))
                          ((:look-location ?look-location))
                          ;; object-in-hand
                          ;; object-hand
                        &allow-other-keys)
  (declare (type desig:object-designator ?source-object ?target-object)
           (type list ?all-sides)
           ;; ?pick-up-robot-location should not be NULL at the beginning
           ;; but can become NULL during execution of the plan
           (type (or desig:location-designator null) ?pour-robot-location))
  "Fetches a perceived object `?object-designator' with
one of arms in the `?arms' lazy list (if not NIL) and one of grasps in `?grasps' if not NIL,
while standing at `?pick-up-robot-location'
and using the grasp and arm specified in `pick-up-action' (if not NIL)."

  (setf ?look-location (desig:reset ?look-location))
  (setf ?pour-robot-location (desig:reset ?pour-robot-location))

  (cpl:with-failure-handling
      ((desig:designator-error (e)
         (roslisp:ros-warn (fd-plans new-pour-mobile) "~a~%Propagating up." e)
         (cpl:fail 'common-fail:fetching-failed
                   :object ?source-object
                   :description "Some designator could not be resolved.")))

    ;; take a new `?pour-robot-location' sample if a failure happens
    (cpl:with-retry-counters ((relocation-for-ik-retries 50))
      (cpl:with-failure-handling
          (((or common-fail:navigation-goal-in-collision
                common-fail:looking-high-level-failure
                common-fail:perception-low-level-failure
                common-fail:object-unreachable
                common-fail:manipulation-low-level-failure
                desig:designator-error) (e)
             (setf ?pour-robot-location
                   (desig:reset ?pour-robot-location))
             (desig:reference ?pour-robot-location)
             (common-fail:retry-with-loc-designator-solutions
                 ?pour-robot-location
                 relocation-for-ik-retries
                 (:error-object-or-string
                  (format NIL "Object of type ~a is unreachable: ~a"
                          (desig:desig-prop-value ?target-object :type) e)
                  :warning-namespace (fd-plans new-pour-mobile)
                  :rethrow-failure 'common-fail:fetching-failed))))

        ;; navigate, look, detect and pick-up
        (exe:perform (desig:an action
                               (type navigating)
                               (location ?pour-robot-location)))

        ;; if fetch location is in hand, we have a handover,
        ;; so move the source hand closer
        ;; (when object-in-hand
        ;;   (let ((?goal
        ;;           (case object-hand
        ;;             (:left `(cpoe:arms-positioned-at :hand-over nil))
        ;;             (:right `(cpoe:arms-positioned-at nil :hand-over))
        ;;             (t `(cpoe:arms-positioned-at nil nil)))))
        ;;     (exe:perform
        ;;      (desig:an action
        ;;                (type positioning-arm)
        ;;                (desig:when (eql object-hand :left)
        ;;                  (left-configuration hand-over))
        ;;                (desig:when (eql object-hand :right)
        ;;                  (right-configuration hand-over))
        ;;                (goal ?goal)))
        ;;     (setf ?look-location (desig:reset ?look-location))))

        (let (;; (?goal `(cpoe:looking-at ,?look-location))
              )
          (exe:perform (desig:an action
                                 (type turning-towards)
                                 (target ?look-location)
                                 ;; (goal ?goal)
                                 )))

        (let ((?more-precise-perceived-target-object
                (exe:perform (desig:an action
                                       (type perceiving)
                                       (object ?target-object)))))

          (let* ((?sides ?all-sides)
                 (?side (cut:lazy-car ?sides)))
            ;; if pouring fails, try another pour orientation
            (cpl:with-retry-counters ((pour-retries 5))
              (cpl:with-failure-handling
                  (((or common-fail:manipulation-low-level-failure
                        common-fail:object-unreachable
                        desig:designator-error) (e)
                     (common-fail:retry-with-list-solutions
                         ?sides
                         pour-retries
                         (:error-object-or-string
                          (format NIL "Pouring failed: ~a.~%Next" e)
                          :warning-namespace (fd-plans new-pour-mobile))
                       (setf ?side (cut:lazy-car ?sides)))))

                ;; (let* ((pour-action
                ;;          (desig:an action
                ;;                    (type pouring-without-retries)
                ;;                    (desig:when ?arm
                ;;                      (arm ?arm))
                ;;                    (desig:when ?side
                ;;                      (side ?side))
                ;;                    (object
                ;;                     ?more-precise-perceived-target-object))))

                ;;   ;; (proj-reasoning:check-picking-up-collisions pour-action)
                ;;   (setf pour-action (desig:current-desig pour-action))

                ;;   (exe:perform pour-action)

                ;;   (desig:current-desig ?target-object))

                (roslisp:ros-info (pp-plans new-pour-mobile)
                                  "Pouring with side: ~a."
                                  ?side)
                (let* ((trajectory
                         (calculate-pour-trajectory
                          ?source-object
                          ?more-precise-perceived-target-object
                          ?side
                          ;; tilt-angle
                          (* pi 2/3)
                          ))
                       (left-reach-pose
                         (when (eq ?arm :left)
                           (first trajectory)))
                       (right-reach-pose
                         (when (eq ?arm :right)
                           (first trajectory)))
                       (left-tilt-down-pose
                         (when (eq ?arm :left)
                           (second trajectory)))
                       (right-tilt-down-pose
                         (when (eq ?arm :right)
                           (second trajectory))))
                  (new-pour-without-retries :arm ?arm
                                            ;; :side ?side
                                            :source-object ?source-object
                                            :target-object ?more-precise-perceived-target-object
                                            ;; :grasp (cut:var-value '?grasp
                                            ;;                       (car (prolog:prolog
                                            ;;                             `(cpoe:object-in-hand
                                            ;;                               ?object
                                            ;;                               ?arm
                                            ;;                               ?grasp))))
                                            :look-pose (or left-tilt-down-pose
                                                           right-tilt-down-pose)
                                            :robot-arm-is-also-a-neck nil
                                            :wait-duration 3
                                            :left-reach-poses (list left-reach-pose)
                                            :right-reach-poses (list right-reach-pose)
                                            :left-tilt-down-poses (list left-tilt-down-pose)
                                            :right-tilt-down-poses (list right-tilt-down-pose)
                                            :left-tilt-up-poses (list left-reach-pose)
                                            :right-tilt-up-poses (list right-reach-pose)))

                )


              )))))))

(defun new-pour-without-retries (&key
                                   ((:source-object ?source-object))
                                   ((:target-object ?target-object))
                                   ;; ((:other-object ?other-object-designator))
                                   ;; other-object-is-a-robot
                                   ((:arm ?arm))
                                   ((:side ?side))
                                   ((:grasp ?grasp))
                                   ;; location-type
                                   ;; ((:gripper-opening ?gripper-opening))
                                   ;; ((:attachment-type ?placing-location-name))
                                   ((:look-pose ?look-pose))
                                   robot-arm-is-also-a-neck
                                   ((:left-reach-poses ?left-reach-poses))
                                   ((:right-reach-poses ?right-reach-poses))
                                   ((:left-tilt-down-poses ?left-tilt-down-poses))
                                   ((:right-tilt-down-poses ?right-tilt-down-poses))
                                   ((:left-tilt-up-poses ?left-tilt-up-poses))
                                   ((:right-tilt-up-poses ?right-tilt-up-poses))
                                   ((:wait-duration ?wait-duration))
                                 &allow-other-keys)
  (declare (ignore ?source-object ?target-object)
           ;; (type desig:object-designator ?source-object ?target-object)
           ;; (type (or desig:object-designator null) ?other-object-designator)
           (type keyword ?arm ;; ?side ?grasp
                 )
           ;; (type (or null keyword) ?placing-location-name)
           (type (or number null) ?wait-duration)
           (type (or null list) ; yes, null is also list, but this is better readable
                 ?left-reach-poses ?right-reach-poses
                 ?left-tilt-down-poses ?right-tilt-down-poses
                 ?left-tilt-up-poses ?right-tilt-up-poses)
           (ignore ?side ?grasp))
  "Reach, tilt-down, wait, tilt-up, park arm."

  (unless robot-arm-is-also-a-neck
    (roslisp:ros-info (new-pour-without-retries) "Looking")
    (cpl:with-failure-handling
        ((common-fail:ptu-low-level-failure (e)
           (roslisp:ros-warn (new-pour-without-retries)
                             "Looking-at had a problem: ~a~%Ignoring."
                             e)
           (return)))
      (exe:perform
       (desig:an action
                 (type looking)
                 (target (desig:a location
                                  (pose ?look-pose)))))))
  (roslisp:ros-info (new-pour-without-retries) "Reaching")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (new-pour-without-retries)
                           "Manipulation messed up: ~a~%Failing."
                           e)
         ;; (return)
         ))
    (let ((?goal `(cpoe:tool-frames-at ,?left-reach-poses ,?right-reach-poses)))
      (exe:perform
       (desig:an action
                 (type reaching)
                 ;; (location ?target-location-designator)
                 (left-poses ?left-reach-poses)
                 (right-poses ?right-reach-poses)
                 (goal ?goal))))
    (let (;; (?goal `(cpoe:gripper-joint-at ,?arm ,?gripper-opening))
        )
    (exe:perform
     (desig:an action
               (type waiting)
               (duration 1)
               ;; (goal ?goal)
               ))))
  (roslisp:ros-info (new-pour-without-retries place) "Tilting down")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (new-pour-without-retries)
                           "Manipulation messed up: ~a~%Failing."
                           e)
         ;; (return)
         ))
    (let ((?goal `(cpoe:tool-frames-at ,?left-tilt-down-poses ,?right-tilt-down-poses)))
      (exe:perform
       (desig:an action
                 (type tilting)
                 ;; (object ?object-designator)
                 ;; (desig:when ?other-object-designator
                 ;;   (supporting-object ?other-object-designator))
                 (left-poses ?left-tilt-down-poses)
                 (right-poses ?right-tilt-down-poses)
                 (goal ?goal)))))
  ;; (when ?placing-location-name
  ;;   (roslisp:ros-info (boxy-plans connect) "Asserting assemblage connection in knowledge base")
  ;;   (if other-object-is-a-robot
  ;;       (cram-occasions-events:on-event
  ;;        (make-instance 'cpoe:object-attached-robot
  ;;          :link (roslisp-utilities:rosify-underscores-lisp-name
  ;;                 (desig:desig-prop-value ?other-object-designator :urdf-name))
  ;;          :not-loose t
  ;;          :object-name (desig:desig-prop-value ?object-designator :name)
  ;;          :other-object-name (or (desig:desig-prop-value ?other-object-designator :name)
  ;;                                 (desig:desig-prop-value ?other-object-designator :part-of))
  ;;          :grasp ?placing-location-name))
  ;;       (cram-occasions-events:on-event
  ;;        (make-instance 'cpoe:object-attached-object
  ;;          :object-name (desig:desig-prop-value ?object-designator :name)
  ;;          :other-object-name (desig:desig-prop-value ?other-object-designator :name)
  ;;          :attachment-type ?placing-location-name))))
  (roslisp:ros-info (new-pour-without-retries) "Waiting")
  (let (;; (?goal `(cpoe:gripper-joint-at ,?arm ,?gripper-opening))
        )
    (exe:perform
     (desig:an action
               (type waiting)
               (duration ?wait-duration)
               ;; (goal ?goal)
               )))
  ;; (roslisp:ros-info (pick-place place) "Retract grasp in knowledge base")
  ;; (cram-occasions-events:on-event
  ;;  (make-instance 'cpoe:object-detached-robot
  ;;    :arm ?arm
  ;;    :object-name (desig:desig-prop-value ?object-designator :name)))
  ;; (roslisp:ros-info (pick-place place) "Updating object location in knowledge base")
  ;; (cram-occasions-events:on-event
  ;;  (make-instance 'cpoe:object-location-changed
  ;;    :object-designator ?object-designator
  ;;    :location-designator ?target-location-designator))
  (roslisp:ros-info (new-pour-without-retries) "Tilting up")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (new-pour-without-retries)
                           "Manipulation messed up: ~a~%Failing."
                           e)
         ;; (return)
         ))
    (let ((?goal `(cpoe:tool-frames-at ,?left-tilt-up-poses ,?right-tilt-up-poses)))
      (exe:perform
       (desig:an action
                 (type tilting)
                 (left-poses ?left-tilt-up-poses)
                 (right-poses ?right-tilt-up-poses)
                 (goal ?goal)))))
  (let (;; (?goal `(cpoe:gripper-joint-at ,?arm ,?gripper-opening))
        )
    (exe:perform
     (desig:an action
               (type waiting)
               (duration 1)
               ;; (goal ?goal)
               )))
  (roslisp:ros-info (new-pour-without-retries) "Parking")
  (exe:perform
   (desig:an action
             (type parking-arms)
             (arms (?arm)))))

(def-fact-group continuous-perception-motions (action-grounding)
  ;; (<- (action-grounding ?action-designator (new-pour )))

  ;; (<- (action-grounding ?action-designator (new-pour-without-retries
  ;;                                           ?resolved-action-designator))
  ;;   (spec:property ?action-designator (:type :new-pouring-without-retries))
  ;;   ;; source
  ;;   (-> (spec:property ?action-designator (:arm ?arm))
  ;;       (-> (spec:property ?action-designator (:source-object
  ;;                                              ?source-designator))
  ;;           (once (or (cpoe:object-in-hand ?source-designator ?arm ?grasp)
  ;;                     (format "WARNING: Wanted to pour from object ~a ~
  ;;                              with arm ~a, but it's not in the arm.~%"
  ;;                             ?source-designator ?arm)))
  ;;           (cpoe:object-in-hand ?source-designator ?arm ?grasp))
  ;;       (-> (spec:property ?action-designator (:source-object
  ;;                                              ?source-designator))
  ;;           (once (or (cpoe:object-in-hand ?source-designator ?arm ?grasp)
  ;;                     (format "WARNING: Wanted to pour from object ~a ~
  ;;                              but it's not in any of the hands.~%"
  ;;                             ?source-designator)))
  ;;           (cpoe:object-in-hand ?source-designator ?arm ?grasp)))
  ;;   (format "1")
  ;;   (desig:current-designator ?source-designator ?current-source-designator)
  ;;   (spec:property ?action-designator (:source-object-side ?source-object-side))
  ;;   ;; destination / target
  ;;   (spec:property ?action-designator (:target-object ?target-designator))
  ;;   (desig:current-designator ?target-designator ?current-target-designator)
  ;;   (once (or (spec:property ?action-designator (:target-object-side ?target-object-side))
  ;;             (equal ?target-object-side :center)))
  ;;   ;; angle
  ;;   (once (or (spec:property ?action-designator (:tilt-angle ?tilt-angle))
  ;;             (equal ?tilt-angle pi)))
  ;;   (format "2")
  ;;   ;; cartesian pouring trajectory
  ;;   (equal ?objects-acted-on (?current-source-designator
  ;;                             ?current-target-designator))
  ;;   (-> (equal ?arm :left)
  ;;       (and (lisp-fun get-pouring-trajectory-in-map
  ;;                      ?arm ?grasp nil ?objects-acted-on
  ;;                      :tilt-angle ?tilt-angle
  ;;                      :source-object-side ?source-object-side
  ;;                      :target-object-side ?target-object-side
  ;;                      ?left-trajectory)
  ;;            (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :reaching
  ;;                      ?left-reach-poses)
  ;;            (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :tilting-down
  ;;                      ?left-tilt-down-poses)
  ;;            (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :tilting-up
  ;;                      ?left-tilt-up-poses)
  ;;            (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :retracting
  ;;                      ?left-retract-poses))
  ;;       (and (equal ?left-reach-poses NIL)
  ;;            (equal ?left-tilt-down-poses NIL)
  ;;            (equal ?left-tilt-up-poses NIL)
  ;;            (equal ?left-retract-poses NIL)))
  ;;   (-> (equal ?arm :right)
  ;;       (and (lisp-fun get-pouring-trajectory-in-map
  ;;                      ?arm ?grasp nil ?objects-acted-on
  ;;                      :tilt-angle ?tilt-angle
  ;;                      :source-object-side ?source-object-side
  ;;                      :target-object-side ?target-object-side
  ;;                      ?right-trajectory)
  ;;            (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :reaching
  ;;                      ?right-reach-poses)
  ;;            (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :tilting-down
  ;;                      ?right-tilt-down-poses)
  ;;            (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :tilting-up
  ;;                      ?right-tilt-up-poses)
  ;;            (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :retracting
  ;;                      ?right-retract-poses))
  ;;       (and (equal ?right-reach-poses NIL)
  ;;            (equal ?right-tilt-down-poses NIL)
  ;;            (equal ?right-tilt-up-poses NIL)
  ;;            (equal ?right-retract-poses NIL)))
  ;;   (once (or (lisp-pred identity ?left-trajectory)
  ;;             (lisp-pred identity ?right-trajectory)))
  ;;   (format "3")
  ;;   ;; wait duration
  ;;   (-> (spec:property ?action-designator (:wait-duration ?wait-duration))
  ;;       (true)
  ;;       (equal ?wait-duration 0))
  ;;   ;; look pose
  ;;   (-> (lisp-pred identity ?left-grasp-poses)
  ;;       (equal ?left-tilt-down-poses (?look-pose . ?_))
  ;;       (equal ?right-tilt-down-poses (?look-pose . ?_)))
  ;;   ;; should look or not?
  ;;   (rob-int:robot ?robot)
  ;;   (-> (man-int:robot-arm-is-also-a-neck ?robot ?arm)
  ;;       (equal ?robot-arm-is-also-a-neck T)
  ;;       (equal ?robot-arm-is-also-a-neck NIL))
  ;;   (format "4")
  ;;   ;; put together resulting designator
  ;;   (desig:designator :action ((:type :pouring)
  ;;                              (:source-object ?current-source-designator)
  ;;                              (:arm ?arm)
  ;;                              (:source-object-side ?source-object-side)
  ;;                              (:grasp ?grasp)
  ;;                              (:target-object ?current-target-designator)
  ;;                              (:target-object-side ?target-object-side)
  ;;                              ;; (:other-object-is-a-robot ?other-object-is-a-robot)
  ;;                              (:look-pose ?look-pose)
  ;;                              (:robot-arm-is-also-a-neck ?robot-arm-is-also-a-neck)
  ;;                              (:wait-duration ?wait-duration)
  ;;                              (:left-reach-poses ?left-reach-poses)
  ;;                              (:right-reach-poses ?right-reach-poses)
  ;;                              (:left-tilt-down-poses ?left-tilt-down-poses)
  ;;                              (:right-tilt-down-poses ?right-tilt-down-poses)
  ;;                              (:left-tilt-up-poses ?left-tilt-up-poses)
  ;;                              (:right-tilt-up-poses ?right-tilt-up-poses)
  ;;                              (:left-retract-poses ?left-retract-poses)
  ;;                              (:right-retract-poses ?right-retract-poses))
  ;;                     ?resolved-action-designator))
  )
