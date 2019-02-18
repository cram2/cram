;;;
;;; Copyright (c) 2018, Alina Hawkin <hawkin@cs.uni-bremen.de>
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

(defun move-to-object (?grasping-base-pose ?grasping-look-pose)
  "Moves the robot into the position which the human had when interacting
with an object. The robot is placed at the spot where the human was standing and
is looking at the spot where the object was in Virtual Reality.
?GRASPING-BASE-POSE: The position for the robot base. Aka, where the human feet
were.
This transform is calculated by map-T-camera->map-P-base function
?GRASPING-LOOK-POSE: The position which the object had in Virtual Reality, and
where the robot should be looking at. This position is calculated by the
grasp-look-pose function.
RETURNS: Errors or a successfull movement action of the robot."
  ;; park arms
  (exe:perform
   (desig:an action
             (type positioning-arm)
             (left-configuration park)
             (right-configuration park)))
  ;; move the robot to specified base location
  (exe:perform
   (desig:an action
             (type going)
             (target (desig:a location (pose ?grasping-base-pose)))))
  ;; move the head to look at specified location (most probably that of an obj)
  (exe:perform
   (desig:an action
             (type looking)
             (target (desig:a location (pose ?grasping-look-pose))))))


(defun pick-up-object (?type)
  "Picks up an object of the given type.
`?type' is the type of the object that is to be picked up as a simple symbol.
RETURNS: Errors or an object designator of picked up object"
  (let* ((?obj-type (object-type-filter-prolog ?type))
         (?arm (query-hand ?obj-type)))
    ;; perceive
    (let ((?obj-desig
            (exe:perform
             (desig:an action
                       (type detecting)
                       (object (desig:an object (type ?obj-type)))))))
      ;; print picking up resolved
      (roslisp:ros-info (kvr pick-up-object)
                        "picking-up action got referenced to ~a"
                        (desig:reference
                         (desig:an action
                                   (type picking-up)
                                   (arm ?arm)
                                   (object ?obj-desig))))
      ;; pick up
      (exe:perform
       (desig:an action
                 (type picking-up)
                 (arm ?arm)
                 (object ?obj-desig)))
      ;; assert attachment
      (cram-occasions-events:on-event
       (make-instance 'cpoe:object-attached-robot
         :object-name (desig:desig-prop-value ?obj-desig :name)
         :arm ?arm
         :grasp :human-grasp))

      ?obj-desig)))


(defun fetch-object (?grasping-base-pose ?grasping-look-pose ?type)
  "A plan to fetch an object.
?GRASPING-BASE-POSE: The pose at which the human stood to pick up the object.
?GRASPING-LOOK-POSE: The pose at which the object was standing when picked up,
and at which the robot will look for it.
?TYPE: the type of the object the robot should look for and which to pick up.
RETURNS: The object designator of the object that has been picked up in this plan."
  ;; navigate
  (move-to-object ?grasping-base-pose ?grasping-look-pose)
  ;; pick up
  (pick-up-object ?type))

(defun deliver-object (?placing-base-pose ?placing-look-pose ?place-pose ?obj-desig)
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
  (let* ((?arm (query-hand
                (object-type-filter-prolog
                 (desig:desig-prop-value ?obj-desig :type)))))
    ;; navigate
    (move-to-object ?placing-base-pose ?placing-look-pose)
    ;; place obj
    (exe:perform
     (desig:an action
               (type placing)
               (arm ?arm)
               (object ?obj-desig)
               (target (desig:a location (pose ?place-pose)))))
    ;; park arms
    (exe:perform
     (desig:an action
               (type positioning-arm)
               (left-configuration park)
               (right-configuration park)))))

(defun transport (?grasping-base-pose ?grasping-look-pose
                  ?placing-base-pose ?placing-look-pose ?place-pose ?type)
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
  (let ((?obj-desig (fetch-object
                     ?grasping-base-pose ?grasping-look-pose ?type)))
    ;; deliver the object
    (deliver-object
     ?placing-base-pose ?placing-look-pose ?place-pose ?obj-desig)))
