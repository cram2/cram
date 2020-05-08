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

(in-package :demo)

(defparameter *start-pose* '((-0.35 -0.9 0.0)(0 0 -0.707 0.707)))
(defparameter *pouring-popcorn-corn-pose* '((-0.2 -0.9 0.0)(0 0 -0.707 0.707)))
(defparameter *robot-salt-pose* '((0.3 -0.9 0)(0 0 -0.707 0.707)))
(defparameter *pot-cooking-pose* '((-0.15 -1.45 0.755)(0 0 0 1)))
(defparameter *pot-lid-on-cooking-pot-pose* '((-0.135 -1.535 0.815)(0 0 0 1)))
(defparameter *pot-lid-after-cooking-pose* '((-0.87 -1.5 0.75)(0 0 0 1)))
(defparameter *ikea-bowl-pose* '((-1.08 -1.44 0.755)(0 0 0 1)))
(defparameter *ikea-plate-pose* '((0.3 -1.52 0.75)(0 0 0 1)))


(defparameter *object-grasps*
  '((:popcorn-pot . :top)
    (:popcorn-pot-lid . :top)
    (:ikea-bowl-ww . :top)
    (:ikea-plate . :top)
    (:salt . :top)))

(defun convert-pose (pose-as-list)
  (cl-tf:pose->pose-stamped cram-tf:*fixed-frame*
                            0.0
                            (btr:ensure-pose pose-as-list)))

(cpl:def-cram-function park-robot ()
  (cpl:with-failure-handling
      ((cpl:plan-failure (e)
         (declare (ignore e))
         (return)))
    (cpl:par
      (exe:perform
       (desig:an action
                 (type positioning-arm)
                 (left-configuration park)
                 (right-configuration park)))
      (let ((?pose (cl-transforms-stamped:make-pose-stamped
                    cram-tf:*fixed-frame*
                    0.0
                    (cl-transforms:make-identity-vector)
                    (cl-transforms:make-identity-rotation))))
        (exe:perform
         (desig:an action
                   (type going)
                   (target (desig:a location
                                    (pose ?pose))))))
      (exe:perform (desig:an action (type opening-gripper) (gripper (left right))))
      (exe:perform (desig:an action (type looking) (direction forward))))))

(defun initialize ()
  (sb-ext:gc :full t)

  ;;(when ccl::*is-logging-enabled*
  ;;    (setf ccl::*is-client-connected* nil)
  ;;    (ccl::connect-to-cloud-logger)
  ;;    (ccl::reset-logged-owl))

  ;; (setf proj-reasoning::*projection-checks-enabled* t)

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
  ;; (setf proj-reasoning::*projection-reasoning-enabled* nil)

  ;;(when ccl::*is-logging-enabled*
  ;;  (ccl::export-log-to-owl "ease_milestone_2018.owl")
  ;;  (ccl::export-belief-state-to-owl "ease_milestone_2018_belief.owl"))
  (sb-ext:gc :full t))

(defun robot-state-changed ()
  (cram-occasions-events:on-event
   (make-instance 'cram-plan-occasions-events:robot-state-changed)))


(cpl:def-cram-function demo ()

  (initialize)
  (when cram-projection:*projection-environment*
    (spawn-objects))

  (park-robot)

  (cpl:with-failure-handling
      ((common-fail:high-level-failure (e)
         (roslisp:ros-warn (pp-plans demo) "Failure happened: ~a~%Skipping..." e)
         (return)))

    ;; Get the pot
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    ;; Pick the popcorn pot up
    (go-to-pose (convert-pose *start-pose*))

    ;; Update robot state
    (robot-state-changed)
    
    (pick-object :popcorn-pot :right)

    ;; Place it on the stove
    (place-object (convert-pose *pot-cooking-pose*) :right)

    ;; Get the ikea bowl with the corn inside and pour corn in the popcorn-pot
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    ;; open the right drawer
    (open-drawer :right)

    ;; pick the ikea bowl up
    (pick-object :ikea-bowl-ww :right)

    ;; go to pose to pour the popcorn corn in the bowl
    (go-to-pose (convert-pose *pouring-popcorn-corn-pose*))

    (robot-state-changed)
    
    ;; pour it in the popcorn-pot
    (let ((?object-to-pour-into
            (world-state-detecting :popcorn-pot)))

      (exe:perform
       (desig:an action
                 (type pouring)
                 (object ?object-to-pour-into)
                 (arms (right))
                 (grasp right-side))))

    ;; go back to the previous pose
    (go-to-pose (convert-pose *start-pose*))
    
    ;; put ikea bowl away
    (place-object (convert-pose *ikea-bowl-pose*) :right)

    ;; Turn the stove on by rotation the knob 1 with the right arm
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    
    ;; open the gripper
    (let ((?arm :right)
          (?gripper-opening 0.1))
      (exe:perform
       (desig:an action
                 (type setting-gripper)
                 (gripper ?arm)
                 (position ?gripper-opening))))
    
    ;; move right arm to knob-1
    (let ((?knob-1-reach-pose
            (cl-tf:make-pose-stamped
             cram-tf:*fixed-frame*
             0.0
             (cl-tf:make-3d-vector -0.535 -1.41 0.754)
             (cl-tf:make-quaternion 0.5 0.5 -0.5 0.5))))
      (move-arms :?right-arm-pose ?knob-1-reach-pose))
    
    ;; close the gripper
    (let ((?arm :right)
          (?gripper-opening 0.018))
      (exe:perform
       (desig:an action
                 (type setting-gripper)
                 (gripper ?arm)
                 (position ?gripper-opening))))
    
    ;; rotate the gripper
    (loop for degree from 0 to 90 by 10 do        
      (let ((?knob-1-rotate-pose
              (cl-tf:make-pose-stamped
               cram-tf:*fixed-frame*
               0.0
               (cl-tf:make-3d-vector -0.535 -1.41 0.754)
               (cl-tf:euler->quaternion :ax (/ 3.14 2) :ay (/ 3.14 2)
                                        :az (* -1 
                                               (cram-math:degrees->radians (+
                                                                            degree
                                                                            180)))))))
        (exe:perform
         (desig:a motion
                  (type moving-tcp)
                  (right-pose ?knob-1-rotate-pose))))
      (setf (btr:joint-state (btr:get-environment-object)
                             "iai_popcorn_stove_knob_1_joint")
            (* -1 
               (cram-math:degrees->radians degree))))

    ;; Attach the lid of the pot to the pot
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    
    ;; pick  up the lid
    (pick-object :popcorn-pot-lid :right)
    
    ;; place it on the pot
    (let* ((?perceived-object-to-place
             (perceive-object :popcorn-pot-lid))
           (?perceived-object-placed-on
             (perceive-object :popcorn-pot))
           (?pose
             (convert-pose *pot-lid-on-cooking-pot-pose*)))
      (place-object ?pose :right 
                    :?object-placed-on ?perceived-object-placed-on
                    :?object-to-place ?perceived-object-to-place
                    :?attachment :popcorn-pot-lid-attachment))
    
    ;; closing the right drawer
    (close-drawer :right)

    ;; Get the ikea plate
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    ;; opening the left drawer
    (open-drawer :left)
    
    ;; TODO: grasping the plate and putting it on the table
    
    ;; closing the left drawer
    (close-drawer :left)
    
    ;; Turn the stove off
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    ;; closing the gripper
    (let ((?arm :right)
          (?gripper-opening 0.018))
      (exe:perform
       (desig:an action
                 (type setting-gripper)
                 (gripper ?arm)
                 (position ?gripper-opening))))
    
    ;; turning the knob 1 off
    (loop for degree from 0 to 90 by 10 do        
      (let ((?knob-1-rotate-pose
              (cl-tf:make-pose-stamped
               cram-tf:*fixed-frame*
               0.0
               (cl-tf:make-3d-vector -0.535 -1.41 0.754)
               (cl-tf:euler->quaternion :ax (/ 3.14 2) :ay (/ 3.14 2)
                                        :az (* -1 
                                               (cram-math:degrees->radians
                                                (-
                                                 90
                                                 degree)))))))
        (exe:perform
         (desig:a motion
                  (type moving-tcp)
                  (right-pose ?knob-1-rotate-pose))))
      (setf (btr:joint-state (btr:get-environment-object)
                             "iai_popcorn_stove_knob_1_joint")
            (* -1 
               (cram-math:degrees->radians (- 90 degree)))))

    ;; Remove the popcorn-pot from the hot stove area with both arms
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    ;; Open the grippers of the robot arms
    (let ((?arm :right)
          (?gripper-opening 0.018))
      (exe:perform
       (desig:an action
                 (type setting-gripper)
                 (gripper ?arm)
                 (position ?gripper-opening))))
    (let ((?arm :left)
          (?gripper-opening 0.018))
      (exe:perform
       (desig:an action
                 (type setting-gripper)
                 (gripper ?arm)
                 (position ?gripper-opening))))
    
    ;; Move arms to the handles of the popcorn pot
    (move-arms 
     :?right-arm-pose
     (convert-pose 
      '((-0.265 -1.53 0.782)
        (0.683 0.183 -0.183 0.683)))
     :?left-arm-pose
     (convert-pose 
      '((-0.005 -1.53 0.782)
        (-0.183 0.683 0.683 0.183))))

    ;; Attaching popcorn pot to one of the robot arm links
    (btr:attach-object (btr:get-robot-object) 
                       (btr:object btr:*current-bullet-world* :popcorn-pot-1)
                       :link "l_gripper_palm_link")

    ;; Move popcorn pot away 
    (move-arms 
     :?right-arm-pose
     (convert-pose 
      '((-0.515 -1.53 0.782)
        (0.683 0.183 -0.183 0.683)))
     :?left-arm-pose
     (convert-pose 
      '((-0.245 -1.53 0.782)
        (-0.183 0.683 0.683 0.183))))

    ;; Detaching popcorn pot from the robot
    (btr:detach-all-objects (btr:get-robot-object))

    ;; Park arms
    (park-arms)
    
    ;; Pour popcorn in the plate
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    ;; Taking popcorn pot lid off
    (pick-object :popcorn-pot-lid :right)
    
    ;; Placing popcorn pot lid on the table
    (place-object (convert-pose *pot-lid-after-cooking-pose*)
                  :right)

    ;; Grasp the popcorn pot
    (move-arms 
     :?right-arm-pose
     (convert-pose 
      '((-0.515 -1.53 0.782)
        (-0.7071067690849304d0 0.0d0 0.0d0 0.7071067690849304d0)))
     :?left-arm-pose
     (convert-pose 
      '((-0.245 -1.53 0.782)
        (0.0d0 0.7071067690849304d0 -0.7071067690849304d0 0.0d0))))


    ;; Attaching popcorn pot to the robot
    (btr:attach-object (btr:get-robot-object) 
                       (btr:object btr:*current-bullet-world* :popcorn-pot-1)
                       :link "l_gripper_palm_link")

    ;; Lift the popcorn pot from the stove
    (move-arms 
     :?right-arm-pose
     (convert-pose 
      '((-0.515 -1.48 1.032)
        (-0.7071067690849304d0 0.0d0 0.0d0 0.7071067690849304d0)))
     :?left-arm-pose
     (convert-pose 
      '((-0.245 -1.48 1.032)
        (0.0d0 0.7071067690849304d0 -0.7071067690849304d0 0.0d0))))

    ;; Move the robot to the plate
    (go-to-pose (convert-pose *robot-salt-pose*) T)

    (robot-state-changed)
    
    ;; Pour the popcorn from the popcorn-pot in the plate
    (let ((?plate-to-pour-into
            (world-state-detecting :plate)))

      (exe:perform
       (desig:an action
                 (type pouring)
                 (object ?plate-to-pour-into)
                 (arms (right left))
                 (grasp back))))

    ;; Go back to the stove the place the popcorn pot
    (go-to-pose (convert-pose *start-pose*) T)

    ;; Move popcorn pot away 
    ;; (move-arms                          
    ;;  :?right-arm-pose
    ;;  (convert-pose 
    ;;   '((-0.515 -1.53 0.782)
    ;;     (0.683 0.183 -0.183 0.683)))
    ;;  :?left-arm-pose
    ;;  (convert-pose 
    ;;   '((-0.245 -1.53 0.782)
    ;;     (-0.183 0.683 0.683 0.183))))

    ;; move popcorn pot above the stove
    (move-arms 
     :?right-arm-pose
     (convert-pose 
      '((-0.515 -1.53 0.782)
        (-0.7071067690849304d0 0.0d0 0.0d0 0.7071067690849304d0)))
     :?left-arm-pose
     (convert-pose 
      '((-0.245 -1.53 0.782)
        (0.0d0 0.7071067690849304d0 -0.7071067690849304d0 0.0d0))))

    ;; Detach popcorn pot from the robot
    (btr:detach-all-objects (btr:get-robot-object))

    (park-arms)

    ;; Salting the popcron
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    ;; move robot to the plate
    (go-to-pose (convert-pose *robot-salt-pose*) T)
    
    (robot-state-changed)
    
    ;; Pick the salt up
    (pick-object :salt :left)

    ;; Salt the popcorn
    (let ((right-hand-position (cl-tf:make-3d-vector 0.28 -1.5 0.86))
          (left-hand-position (cl-tf:make-3d-vector 0.28 -1.5 0.9))
          (offset 0.01))
      
      (dotimes (c 5)
        (let ((dynamic-right-hand-position 
                (with-slots (cl-tf:x cl-tf:y cl-tf:z)
                    right-hand-position
                  (cl-tf:make-3d-vector (+ cl-tf:x (* c offset))
                                        cl-tf:y
                                        cl-tf:z)))
              (dynamic-left-hand-position 
                (with-slots (cl-tf:x cl-tf:y cl-tf:z)
                    left-hand-position
                  (cl-tf:make-3d-vector (+ cl-tf:x (* c offset))
                                        cl-tf:y
                                        cl-tf:z))))

          (when (eq c 0)
            (let ((?arm :right)
                  (?gripper-opening 0.048))
              (exe:perform
               (desig:an action
                         (type setting-gripper)
                         (gripper ?arm)
                         (position ?gripper-opening)))))

          ;; Moving the salt in the right pose over the plate
          (move-arms 
           :?right-arm-pose
           (cl-tf:make-pose-stamped
            cram-tf:*fixed-frame*
            0.0
            dynamic-right-hand-position 
            (cl-tf:euler->quaternion :ax pi :ay 0.0 :az 0))
           :?left-arm-pose
           (cl-tf:make-pose-stamped
            cram-tf:*fixed-frame*
            0.0
            dynamic-left-hand-position
            (cl-tf:euler->quaternion :ax pi :ay 0.0 :az pi)))

          (sleep 0.5)
          
          ;; Salt the pocorn by ....
          ;; ... rotating both endeffectors towards the robot
          (dotimes (angle-c 6)
            (move-arms 
             :?right-arm-pose
             (cl-tf:make-pose-stamped
              cram-tf:*fixed-frame*
              0.0
              dynamic-right-hand-position 
              (cl-tf:euler->quaternion :ax pi :ay 0.0 :az (*
                                                           angle-c
                                                           (/ pi 36))))
             :?left-arm-pose
             (cl-tf:make-pose-stamped
              cram-tf:*fixed-frame*
              0.0
              dynamic-left-hand-position
              (cl-tf:euler->quaternion :ax pi :ay 0.0 :az (- pi (*
                                                                 angle-c
                                                                 (/ pi 36)))))))

          (sleep 0.5)
          ;; ... and rotating back.
          (sleep 0.5)
          ;; repeat for another pose above the plate
          )))
    
    ;; place salt back
    (place-object (convert-pose '((0.5 -1.6 0.78) (0 0 0 1))) :left)

    (park-robot))
  
  
  (park-robot)
  
  (finalize)
  
  cpl:*current-path*)
