;;;
;;; Copyright (c) 2019, Thomas Lipps <tlipps@uni-bremen.de>
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

(in-package :btr-tests)

(defun clean-environment (from)
  (mapcar (lambda (name)
            (btr:remove-object btr:*current-bullet-world* name))
          from))

(defun spawn-objects-with-same-pose (names)
  (mapcar (lambda (name)
            (btr-utils:spawn-object name :mug :pose '((0 0 2.0) (0 0 0 1))))
          names))

(define-test robot-attached-objects-in-collision-negative
  ;; Tests if the function returns nil since not one object is attached
  ;; to a robot link.
  (setup-world)
  (spawn-objects-with-same-pose '(o oo))
  (assert-false (btr:robot-attached-objects-in-collision)) ;; -> no collision
  (clean-environment '(o oo)))

(define-test robot-attached-objects-in-collision-positive
  ;; Tests if the function registers a collision since one of the objects
  ;; is attached to the robot and the other is in collision to the
  ;; attached object.
  (setup-world)
  (spawn-objects-with-same-pose '(o oo))
  (btr:attach-object (btr:get-robot-object)
                     (btr:object btr:*current-bullet-world* 'o)
                     :link "base_footprint")
  (assert-true (btr:robot-attached-objects-in-collision)) ;; -> collision
  (clean-environment '(o oo)))

(define-test robot-attached-objects-in-collision-negative-because-of-attachment
  ;; Since the robot can attach things made of items attached to each other,
  ;; there should not be a collision if the robot attaches one of
  ;; these things. Even if the items in the "thing" are in collision
  ;; to each other.
  (setup-world)
  (spawn-objects-with-same-pose '(o oo))
  ;; "thing" 'o: 'oo is attached to 'o with the same pose, therefore
  ;; they are in collision.
  (btr:attach-object 'o 'oo)
  ;; attach "thing" 'o to a robot link
  (btr:attach-object (btr:get-robot-object)
                     (btr:object btr:*current-bullet-world* 'o)
                     :link "base_footprint")
  (assert-false (btr:robot-attached-objects-in-collision)) ;; -> no collision
  (clean-environment '(o oo)))

(define-test robot-attached-objects-in-collision-positive-colliding-with-floor
  (setup-world)
  (btr-utils:kill-all-objects)
  (btr:detach-all-objects (btr:get-robot-object))
  (btr:detach-all-objects (btr:get-environment-object))
  ;; spawn plate intersecting the floor
  (btr-utils:spawn-object :plate-1 :plate :pose '((0.9 0 0.0) (0.7 0 0 0.7)) :mass 1.0)
  ;; attach plate to the robot
  (btr:attach-object (btr:get-robot-object)
                     (btr:object btr:*current-bullet-world* :plate-1)
                     :link "r_wrist_roll_link" :loose nil :grasp :front)
  ;; check for collisions
  (assert-equal '(:FLOOR) (mapcar #'btr:name (btr::robot-attached-objects-in-collision)))
  (clean-environment '(:plate-1)))

(define-test robot-attached-objects-in-collision-positive-environment-collisions
  (setup-world)
  (spawn-robot)
  (btr-utils:kill-all-objects)
  (btr:detach-all-objects (btr:get-robot-object))
  (btr:detach-all-objects (btr:get-environment-object))
  ;; spawn basket which is in collision with the environment
  (btr:add-object btr:*current-bullet-world* :basket :basket-1
                  '((1.2 0 0.8) (0.0 0 0 1.0))
                                                     :length 0.2 :width 0.2 :height 0.2)
  ;; attach basket to robot 
  (btr:attach-object (btr:get-robot-object)
                     (btr:object btr:*current-bullet-world* :basket-1)
                     :link "r_wrist_roll_link" :loose nil :grasp :front)
  ;; check for collision 
  (assert-equal '(:ENVIRONMENT) 
                (mapcar #'btr:name (btr::robot-attached-objects-in-collision)))
  ;; spawn plate on sink area surface ...
  (btr-utils:spawn-object :plate-1 :plate :pose '((1.5 0.5 0.9) (0.0 0 0 1.0)))
  ;; ... and attach it to the environment 
  (btr:attach-object (btr:get-environment-object)
                     (btr:object btr:*current-bullet-world*
                                 :plate-1)
                     :link "sink_area_surface")
  ;; and check for collisions again 
  (assert-equal '(:ENVIRONMENT) 
                (mapcar #'btr:name (btr::robot-attached-objects-in-collision)))
  (clean-environment '(:basket-1 :plate-1)))

(define-test robot-attached-objects-in-collision-negative-environment-collisions
  (setup-world)
  (spawn-robot)
  (btr-utils:kill-all-objects)
  (btr:detach-all-objects (btr:get-robot-object))
  (btr:detach-all-objects (btr:get-environment-object))
  ;; spawn basket which is in collision with the environment
  (btr:add-object btr:*current-bullet-world* :basket :basket-1
                  '((1.2 0 0.8) (0.0 0 0 1.0))
                                                     :length 0.2 :width 0.2 :height 0.2)
  ;; attach basket to robot 
  (btr:attach-object (btr:get-robot-object)
                     (btr:object btr:*current-bullet-world* :basket-1)
                     :link "r_wrist_roll_link" :loose nil :grasp :front)
  ;; spawn plate on sink area surface ...
  (btr-utils:spawn-object :plate-1 :plate :pose '((1.5 0.5 0.9) (0.0 0 0 1.0)))
  ;; ... and attach it to the environment 
  (btr:attach-object (btr:get-environment-object)
                     (btr:object btr:*current-bullet-world*
                                 :plate-1)
                     :link "sink_area_surface")
  ;; attach basket to the environment 
  (btr:attach-object (btr:get-environment-object)
                     (btr:object btr:*current-bullet-world*
                                 :basket-1)
                     :link "sink_area_surface")
  ;; check collisions again 
  (assert-false (mapcar #'btr:name (btr::robot-attached-objects-in-collision)))
  (clean-environment '(:basket-1 :plate-1)))
