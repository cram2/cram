;;;
;;; Copyright (c) 2020, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     Thomas Lipps    <tlipps@uni-bremen.de>
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

;; ***************************************************************************
;; ****************************** ROBOT POSES ********************************
;; ***************************************************************************

(defparameter *start-pose* '("iai_popcorn_table_footprint" . ((-0.05 0.77 0.0)(0.0 0.0 -0.707 0.707))))
(defparameter *pouring-popcorn-corn-pose* '("iai_popcorn_table_footprint" . ((0.1 0.77 0.0)(0.0d0 0.0d0 -0.707 0.707))))
(defparameter *robot-salt-pose* '("iai_popcorn_table_footprint" . ((0.4 0.77 0.0)(0.0 0.0 -0.707 0.707))))

;; ***************************************************************************
;; ************************** ITEM SPAWNING POSES ****************************
;; ***************************************************************************

(defparameter *popcorn-pot-init-pose* '("iai_popcorn_table_surface" . ((-0.575 0.22 -0.505)(0 0 0 1))))
(defparameter *salt-init-pose* '("iai_popcorn_table_surface" . ((0.8d0 0.070 0.08)(0 0 0 1))))
(defparameter *plate-init-pose* '("iai_popcorn_table_surface" . ((0.6 0.15 0.05)(0 0 0 1))))
(defparameter *ikea-plate* '("iai_popcorn_table_surface" . ((0.12 1.32 -0.60)(0 0 0 1))))

;; ***************************************************************************
;; ******************** ITEM POSES DURING POPCORN MAKING *********************
;; ***************************************************************************

;; ****************************** PLACING POSES ******************************
(defparameter *pot-cooking-pose* '("iai_popcorn_table_surface" . ((0.15 0.22 0.055)(0 0 0 1))))
(defparameter *pot-lid-on-cooking-pot-pose* '("iai_popcorn_table_surface" . ((0.165 0.135 0.115)(0 0 0 1))))
(defparameter *pot-lid-after-cooking-pose* '("iai_popcorn_table_surface" . ((-0.55 0.17 0.05) (0 0 0 1))))
(defparameter *ikea-bowl-placing-pose* '("iai_popcorn_table_surface" . ((-0.78 0.23 0.055)(0 0 0 1))))
(defparameter *ikea-plate-placing-pose* '("iai_popcorn_table_surface" . ((0.6 0.20 0.05)(0 0 0 1))))
(defparameter *ikea-plate-pose* '("iai_popcorn_table_surface" . ((0.6 0.15 0.05)(0 0 0 1))))
;;(defparameter *popcorn-pot-away-from-hot-stove-right-diagonal* '("iai_popcorn_table_surface" . ((-0.215 0.14 0.082)(0.68 0.183 -0.183 0.68))))
;;(defparameter *popcorn-pot-away-from-hot-stove-left-diagonal* '("iai_popcorn_table_surface" . ((0.055 0.14 0.082)(-0.183 0.683 0.683 0.183))))
;;(defparameter *popcorn-pot-away-from-hot-stove-right-horizontal* '("iai_popcorn_table_surface" . ((-0.215 0.14 0.082)(-0.707 0.0d0 0.0d0 0.707))))
;;(defparameter *popcorn-pot-away-from-hot-stove-left-horizontal* '("iai_popcorn_table_surface" . ((0.055 0.14 0.082)(0.0d0 0.707 -0.707 0.0d0))))
(defparameter *popcorn-pot-away-from-hot-stove-horizontal* '("iai_popcorn_table_surface" . ((-0.08 0.14 0.082)(0.0d0 0.0d0 0.0d0 1.0d0))))
;;(defparameter *popcorn-pot-lifting-from-table-right* '("iai_popcorn_table_surface" . ((-0.215 0.14 0.332)(-0.707 0.0d0 0.0d0 0.707))))
;;(defparameter *popcorn-pot-lifting-from-table-left* '("iai_popcorn_table_surface" . ((0.055 0.14 0.332)(0.0d0 0.707 -0.707 0.0d0))))

;; ****************************** PICKING POSES ******************************
(defparameter *ikea-bowl-picking-pose* '("iai_popcorn_table_surface" . ((-0.65 0.42 -0.13)(0 0 0 1))))
(defparameter *ikea-plate-picking-pose* '("iai_popcorn_table_surface" . ((0.601 0.5065 -0.147)(0 0 0 1))))
(defparameter *popcorn-pot-lid-picking-pose* '("iai_popcorn_table_surface" . ((-0.48 0.54 -0.14)(0 0 0 1))))
(defparameter *popcorn-pot-lid-on-popcorn-pot* '("iai_popcorn_table_surface" . ((-0.06 0.14 0.125)(0 0 0 1))))
(defparameter *salt-picking-pose* '("iai_popcorn_table_surface" . ((0.8 0.07 0.08)(0 0 0 1))))

;; ****************************** ARM POSES **********************************
(defparameter *knob-1-reaching-pose* '("iai_popcorn_stove_knob_1" . ((0.0 0.0 0.034)(-0.5 -0.5 0.5 -0.5))))
(defparameter *popcorn-pot-handle-right-diagonal-grasp* '(:popcorn-pot-1 . ((-0.128 -0.004 0.0309)(0.68 0.183 -0.183 0.68))))
(defparameter *popcorn-pot-handle-left-diagonal-grasp* '(:popcorn-pot-1 . ((0.1315 0.0114 0.031)(-0.183 0.683 0.683 0.183))))
(defparameter *popcorn-pot-handle-right-horizontal-grasp* '(:popcorn-pot-1 . ((-0.128 -0.004 0.0309)(-0.707 0.0d0 0.0d0 0.707))))
(defparameter *popcorn-pot-handle-left-horizontal-grasp* '(:popcorn-pot-1 . ((0.1315 0.0114 0.031)(0.0d0 0.707 -0.707 0.0d0))))

(defparameter *with-logging* nil)

(defun initialize ()
  (sb-ext:gc :full t)

  (setf proj-reasoning::*projection-checks-enabled* T)

  (btr:detach-all-objects (btr:get-robot-object))
  (btr:detach-all-objects (btr:get-environment-object))
  (btr-utils:kill-all-objects)
  (setf (btr:joint-state (btr:get-environment-object)
                         "iai_popcorn_table_drawer_left_table_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "iai_popcorn_table_drawer_right_table_joint")  
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "iai_popcorn_stove_knob_1_joint")
        0.0)
  (btr-belief::publish-environment-joint-state
   (btr:joint-states (btr:get-environment-object)))

  (setf desig::*designators* (tg:make-weak-hash-table :weakness :key))

  (coe:clear-belief)
  
  (btr:clear-costmap-vis-object))

(defun finalize ()
  (sb-ext:gc :full t))

(defun robot-state-changed ()
  (cram-occasions-events:on-event
   (make-instance 'cram-plan-occasions-events:robot-state-changed)))
  

(cpl:def-cram-function demo ()

  (initialize)
  (when cram-projection:*projection-environment*
    (spawn-objects))

   (when *with-logging*
     (ccl::start-episode))

  (park-robot)

  (cpl:with-failure-handling
      ((common-fail:high-level-failure (e)
         (roslisp:ros-warn (pp-plans demo) "Failure happened: ~a~%Skipping..." e)
         (return)))

    ;; 1. Getting the pot and placing it on the stove
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    ;; Picking the popcorn pot up
    (go-to-pose *start-pose*)

    ;; Updating robot state
    (robot-state-changed)
    
    ;; Picking the popcorn pot with the right arm up
    (pick-object :popcorn-pot '(:right)
                 *popcorn-pot-init-pose*
                 :?right-grasp :top)
    

    ;; Placing it on the stove
    (place-object '(:right)
                  *pot-cooking-pose*
                  :?right-grasp :top)

    ;; 2. Getting the ikea bowl with the corn inside and pour corn in the popcorn pot
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    ;; Opening the right drawer
    (open-drawer :right)

    ;; Picking the ikea bowl with the right arm up
    (pick-object :ikea-bowl-ww '(:right)
                 *ikea-bowl-picking-pose*)

    ;; Going to pose to pour the popcorn corn in the bowl
    (go-to-pose *pouring-popcorn-corn-pose*)

    ;; Updating robot state
    (robot-state-changed)

    ;; Pouring corn from bowl in the popcorn pot
    (pour-into :popcorn-pot '(:right) :right-side)

    ;; Going back to the previous pose
    (go-to-pose *start-pose*)
    
    ;; Putting the ikea bowl away
    (place-object '(:right) *ikea-bowl-placing-pose*
                  :?right-grasp :top)
    

    ;; 3. Turning the stove on by rotation the knob 1 with the right arm
    ;; ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    ;; Rotating the right knob to turn the stove on
    (exe:perform
     (desig:an action 
               (type :turning-knob)
               (arm :right)
               (frame "iai_popcorn_stove_knob_1")
               (configuration :on)))

    ;; 4. Attaching the lid of the pot to the popcorn pot
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    
    ;; Picking up the lid
    (pick-object :popcorn-pot-lid '(:right)
                 *popcorn-pot-lid-picking-pose*)
    
    ;; Placing it on the popcorn pot
    (let* ((pose *pot-lid-on-cooking-pot-pose*)
           (?object-placed-on
             (get-object-designator :popcorn-pot))
           (?object-to-place
             (get-object-designator :popcorn-pot-lid)))
      (place-object '(:right) pose
                    :?right-grasp :top
                    :?object-placed-on ?object-placed-on
                    :?object-to-place ?object-to-place
                    :?attachment :popcorn-pot-lid-attachment))
  
    ;; Closing the right drawer
    (close-drawer :right)

    ;; 5. Getting the ikea plate
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    ;; Opening the left drawer
    (open-drawer :left)

    ;; Grasping the plate
    (pick-object :ikea-plate '(:left)
                 *Ikea-plate-picking-pose*
                 :?left-grasp :top)

    ;; Putting the plate on the table
    (place-object '(:left) *ikea-plate-placing-pose*
                  :?left-grasp :top)
    
    ;; Closing the left drawer
    (close-drawer :left)
    
    ;; 6. Turning the stove off
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    
    ;; Turning the stove off by turning knob 1
    (exe:perform
     (desig:an action 
               (type :turning-knob)
               (arm :right)
               (frame "iai_popcorn_stove_knob_1")
               (configuration :off)))

    ;; 7. Removing the popcorn pot from the hot stove area with both arms
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    ;; Picking the popcorn pot with both arms
    (pick-object :popcorn-pot '(:left :right)
                 nil
                 :?left-grasp :left-side
                 :?right-grasp :right-side)

    ;; Placing the popcorn pot off the hot stove
    (place-object '(:left :right)
                  *popcorn-pot-away-from-hot-stove-horizontal*
                  :?left-grasp :left-side
                  :?right-grasp :right-side)

    ;; Parking arms
    (park-arms)
    
    ;; 8. Pouring popcorn on the plate
    ;; ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    ;; Taking popcorn pot lid off the popcorn pot
    (pick-object :popcorn-pot-lid '(:right)
                 *popcorn-pot-lid-on-popcorn-pot*)
    
    ;; Placing popcorn pot lid on the table
    (place-object '(:right) *pot-lid-after-cooking-pose*
                  :?right-grasp :top)

    
    ;; Picking the popcorn pot with both arms
    (pick-object :popcorn-pot '(:left :right)
                 nil
                 :?left-grasp :left-side
                 :?right-grasp :right-side)

    
    ;; Moving the robot to the plate
    (go-to-pose *robot-salt-pose* :dont-move-arms T)

    ;; Updating robot state
    (robot-state-changed)
    
    ;; Pouring the popcorn from the popcorn pot in the plate
    (pour-into :ikea-plate '(:right :left) :front)

    ;; Going back to the stove to place the popcorn pot
    (go-to-pose *start-pose* :dont-move-arms T)

    ;; Placing the popcorn pot on the stove
    (place-object '(:left :right)
                  *popcorn-pot-away-from-hot-stove-horizontal*
                  :?left-grasp :left-side
                  :?right-grasp :right-side)    

    ;; Parking arms
    (park-arms)

    ;; 9. Salting the popcron
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    ;; Moving robot to the plate
    (go-to-pose *robot-salt-pose* :dont-move-arms T)

    ;; Updating robot state
    (robot-state-changed)
    
    ;; Picking the salt up
    (pick-object :salt '(:left)
                 *salt-picking-pose*)
    
    ;; Salting the popcorn
    (exe:perform
     (desig:an action 
               (type salting)
               (object (desig:an object
                                 (type salt)
                                 (name salt-1)))
               (on-object (desig:an object
                                    (type ikea-plate)
                                    (name ikea-plate-1)))))
    
    ;; Placing salt back
    (place-object '(:left) *salt-picking-pose*
                  :?left-grasp :left-side)

    ;; Parking robot
    (park-robot))
  
  
  (park-robot)
  
  (finalize)

  (when *with-logging*
    (ccl::start-episode))
  
  cpl:*current-path*)
