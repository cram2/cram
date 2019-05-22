;;;
;;; Copyright (c) 2018, Alina Hawkin <hawkin@cs.uni-bremen.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(in-package :kvr)

#+now-using-the-plans-from-pr2-fetch-deliver-plans
(

(defun search-for-object (?base-poses ?look-poses ?object-designator)
  ;; pick one look pose from the lazy list
  (let ((?look-pose (cut:lazy-car ?look-poses)))

    (cpl:with-retry-counters ((outer-look-pose-retries 2))
      (cpl:with-failure-handling
          (((or common-fail:navigation-low-level-failure
                common-fail:navigation-goal-in-collision
                common-fail:perception-low-level-failure) (e)
             (declare (ignore e))
             (roslisp:ros-warn (kvr plans)
                               "Search is about to give up. Next solution.")
             (cpl:do-retry outer-look-pose-retries
               (setf ?look-poses (cut:lazy-cdr ?look-poses))
               (setf ?look-pose (cut:lazy-car ?look-poses))
               (if ?look-pose
                   (cpl:retry)
                   (roslisp:ros-warn (kvr plans) "No more solutions.")))
             (roslisp:ros-warn (kvr plans) "Outer look retry counter empty.")))

        ;; move the robot to specified base location
        ;; pick one of the base-poses from the lazy list
        (let ((?base-pose (cut:lazy-car ?base-poses)))

          ;; if the going action fails, pick another base-pose from the lazy list
          (cpl:with-retry-counters ((nav-pose-retries 20))
            (cpl:with-failure-handling
                (((or common-fail:navigation-low-level-failure
                      common-fail:navigation-goal-in-collision
                      common-fail:perception-low-level-failure) (e)
                   (declare (ignore e))
                   (roslisp:ros-warn (kvr plans) "Navigation failed. Next solution.")
                   (cpl:do-retry nav-pose-retries
                     (setf ?base-poses (cut:lazy-cdr ?base-poses))
                     (setf ?base-pose (cut:lazy-car ?base-poses))
                     (if ?base-pose
                         (cpl:retry)
                         (roslisp:ros-warn (kvr plans) "No more solutions.")))
                   (roslisp:ros-warn (kvr plans) "nav retry counter empty.")))

              ;; navigate
              (exe:perform
               (desig:an action
                         (type navigating)
                         (location (desig:a location (pose ?base-pose)))))

              ;; move the head to look at specified location
              ;; (most probably that of an obj)
              ;; and detect

              ;; if detection fails, try another looking target
              (cpl:with-retry-counters ((look-pose-retries 5))
                (cpl:with-failure-handling
                    ((common-fail:perception-low-level-failure (e)
                       (declare (ignore e))
                       (roslisp:ros-warn (kvr plans)
                                         "Perception failed. Next solution.")
                       (cpl:do-retry look-pose-retries
                         (setf ?look-poses (cut:lazy-cdr ?look-poses))
                         (setf ?look-pose (cut:lazy-car ?look-poses))
                         (if ?look-pose
                             (cpl:retry)
                             (roslisp:ros-warn (kvr plans) "No more solutions.")))
                       (roslisp:ros-warn (kvr plans) "look retry counter empty.")))

                  ;; do the looking
                  (exe:perform
                   (desig:an action
                             (type turning-towards)
                             (target (desig:a location (pose ?look-pose)))))

                  ;; perceive
                  (exe:perform
                   (desig:an action
                             (type detecting)
                             (object ?object-designator))))))))))))




(defun fetch-object (?base-poses ?grasp-types ?arms ?object-designator)
  "A plan to fetch an object.
?GRASPING-BASE-POSE: The pose at which the human stood to pick up the object.
?GRASPING-LOOK-POSE: The pose at which the object was standing when picked up,
and at which the robot will look for it.
?TYPE: the type of the object the robot should look for and which to pick up.
RETURNS: The object designator of the object that has been picked up in this plan."

  ;; move the robot to specified base location
  ;; pick one of the base-poses from the lazy list
  (let ((?base-pose (cut:lazy-car ?base-poses)))

    ;; if the going action fails, pick another base-pose from the lazy list
    (cpl:with-retry-counters ((nav-pose-retries 30))
      (cpl:with-failure-handling
          (((or common-fail:navigation-low-level-failure
                common-fail:perception-low-level-failure
                common-fail:manipulation-low-level-failure
                common-fail:navigation-goal-in-collision) (e)
             (declare (ignore e))
             (roslisp:ros-warn (kvr plans) "Navigation failed. Next solution.")
             (cpl:do-retry nav-pose-retries
               (setf ?base-poses (cut:lazy-cdr ?base-poses))
               (setf ?base-pose (cut:lazy-car ?base-poses))
               (if ?base-pose
                   (cpl:retry)
                   (roslisp:ros-warn (kvr plans) "no more solutions")))
             (roslisp:ros-warn (kvr plans) "nav retry counter empty")))

        ;; come closer to the object
        (exe:perform
         (desig:an action
                   (type navigating)
                   (location (desig:a location (pose ?base-pose)))))

        ;; reperceive from pick-up position
        (let ((?obj-desig
                (exe:perform
                 (desig:an action
                           (type detecting)
                           (object ?object-designator)))))

          (let ((?arm (cut:lazy-car ?arms)))
            ;; if picking up fails, try another arm
            (cpl:with-retry-counters ((arm-retries 1))
              (cpl:with-failure-handling
                  ((common-fail:manipulation-low-level-failure (e)
                     (declare (ignore e))
                     (roslisp:ros-warn (kvr plans) "manipulation failed. Next.")
                     (cpl:do-retry arm-retries
                       (setf ?arms (cut:lazy-cdr ?arms))
                       (setf ?arm (cut:lazy-car ?arms))
                       (if ?base-pose
                           (cpl:retry)
                           (roslisp:ros-warn (kvr plans) "no more solutions")))
                     (roslisp:ros-warn (kvr plans) "arm retry counter empty")))

                (let ((?grasp-type (cut:lazy-car ?grasp-types)))
                  ;; if picking up fails, try another grasp orientation
                  (cpl:with-retry-counters ((grasp-retries 4))
                    (cpl:with-failure-handling
                        ((common-fail:manipulation-low-level-failure (e)
                           (declare (ignore e))
                           (roslisp:ros-warn (kvr plans) "Picking up failed. Next.")
                           (cpl:do-retry grasp-retries
                             (setf ?grasp-types (cut:lazy-cdr ?grasp-types))
                             (setf ?grasp-type (cut:lazy-car ?grasp-types))
                             (if ?grasp-type
                                 (cpl:retry)
                                 (roslisp:ros-warn (kvr plans) "No more sols.")))
                           (roslisp:ros-warn (kvr plans) "grasp retry cntr empty")))

                      ;; pick up
                      (exe:perform
                       (desig:an action
                                 (type picking-up)
                                 (arm ?arm)
                                 (grasp ?grasp-type)
                                 (object ?obj-desig)))))))))

          ?obj-desig)))))




(defun test-placing-pose-stability (object-desig placing-pose)
  (let* ((world
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
           (cpl:sleep urdf-proj::*debug-short-sleep-duration*)
           (btr:simulate btr:*current-bullet-world* 500)
           (btr:simulate btr:*current-bullet-world* 100)
           (let* ((new-pose
                    (btr:pose new-btr-object))
                  (distance-new-pose-and-place-pose
                    (cl-tf:v-dist
                     (cl-transforms:origin new-pose)
                     (cl-transforms:origin placing-pose))))
             (when (> distance-new-pose-and-place-pose 0.2)
               (cpl:fail 'common-fail:high-level-failure
                         :description "Pose unstable."))))
      (btr::restore-world-state world-state world))))

(defun deliver-object (?place-poses ?base-poses ?obj-desig)
  "A plan to place an object which is currently in one of the robots hands.
?PLACING-BASE-POSE: The pose the robot should stand at in order to place the
object. Usually the pose where the human was standing while placing the object
in Virtual Reality.
?PLACING-LOOK-POSE: The pose where the robot looks at while placing the object.
The same pose at which the human placed the object.
?PLACE-POSE: The pose at which the object was placed in Virtual Reality.
Relative to the Kitchen Island table.
?OBJ-DESIG: The object deignator of the the object which the robot currently
holds in his hand and which is to be placed."

  ;; pick one of the place poses from the lazy list
  (let ((?place-pose (cut:lazy-car ?place-poses)))

    ;; if the placing fails, pick another place pose from the lazy list
    (cpl:with-retry-counters ((place-pose-retries 10))
      (cpl:with-failure-handling
          (((or common-fail:navigation-low-level-failure
                common-fail:manipulation-low-level-failure
                common-fail:high-level-failure) (e)
             (declare (ignore e))
             (roslisp:ros-warn (kvr plans) "Placing failed. Next solution.")
             (cpl:do-retry place-pose-retries
               (setf ?place-poses (cut:lazy-cdr ?place-poses))
               (setf ?place-pose (cut:lazy-car ?place-poses))
               (if ?place-pose
                   (cpl:retry)
                   (roslisp:ros-warn (kvr plans) "No more solutions.")))
             (roslisp:ros-warn (kvr plans) "place retry counter empty.")))

        ;; test if the placing pose is a good one -- not falling on the floor
        ;; test function throws a high-level-failure if not good pose
        (test-placing-pose-stability ?obj-desig ?place-pose)

        ;; move the robot to specified base location
        ;; pick one of the base-poses from the lazy list
        (let ((?base-pose (cut:lazy-car ?base-poses)))

          ;; if the going action fails, pick another base-pose from the lazy list
          (cpl:with-retry-counters ((nav-pose-retries 20))
            (cpl:with-failure-handling
                (((or common-fail:navigation-low-level-failure
                      common-fail:manipulation-low-level-failure) (e)
                   (declare (ignore e))
                   (roslisp:ros-warn (kvr plans) "Nav failed. Next solution.")
                   (cpl:do-retry nav-pose-retries
                     (setf ?base-poses (cut:lazy-cdr ?base-poses))
                     (setf ?base-pose (cut:lazy-car ?base-poses))
                     (if ?base-pose
                         (cpl:retry)
                         (roslisp:ros-warn (kvr plans) "No more solutions.")))
                   (roslisp:ros-warn (kvr plans) "nav retry counter empty.")))

              ;; navigate
              (exe:perform
               (desig:an action
                         (type navigating)
                         (location (desig:a location (pose ?base-pose)))))

              (exe:perform
               (desig:an action
                         (type looking)
                         (target (desig:a location (pose ?place-pose)))))

              ;; place obj
              (exe:perform
               (desig:an action
                         (type placing)
                         (object ?obj-desig)
                         (target (desig:a location (pose ?place-pose))))))))))))


(defun transport (?searching-base-poses ?searching-look-poses ?grasping-base-poses
                  ?grasp-types
                  ?placing-base-poses ?placing-look-poses ?place-poses ?type)
  "Picks up and object and places it down based on Virtual Reality data.
?GRASPING-BASE-POSE: The pose the robot should stand at, in order to be able to
grasp the object.
?GRASPING-LOOK-POSE: The pose the robot is going to look at, in order to look
for the object to be picked up.
?PLACING-BASE-POSE: The pose where the robot should stand in order to be able
to place down the picked up object.
?PLACING-LOOK-POSE: The pose the robot is looking at, at which he will place
the object.
?PLACE-POSE: The actual placing pose of the object.
?TYPE: The type of the object the robot should interact with."
  ;; fetch the object
  (let* ((?found-obj-desig (search-for-object
                            ?searching-base-poses ?searching-look-poses ?type))
         (?fetched-obj-desig (fetch-object
                              ?grasping-base-poses ?grasp-types ?type)))
    ;; deliver the object
    (deliver-object
     ?placing-base-poses ?placing-look-poses ?place-poses ?fetched-obj-desig ?type)))
)
