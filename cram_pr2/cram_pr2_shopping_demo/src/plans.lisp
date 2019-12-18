;;;
;;; Copyright (c) 2019, Jonas Dech <jdech[at]uni-bremen.de
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
;;;     * Neither the name of the Institute for Artificial Intelligence /
;;;       University of Bremen nor the names of its contributors
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

(in-package :cram-pr2-shopping-demo)

(defparameter *table* (cl-transforms-stamped:make-pose-stamped
                       "map" 0.0
                       (cl-transforms:make-3d-vector 1 0.9 0.7)
                       (cl-transforms:make-identity-rotation)))

(defparameter *pose-searching* (cl-transforms-stamped:make-pose-stamped
                                "map" 0
                                (cl-transforms:make-3d-vector 1 0.7 1)
                                (cl-transforms:make-identity-rotation)))

(defparameter *placing-pose* (cl-transforms-stamped:make-pose-stamped
                              "map" 0
                              (cl-transforms:make-3d-vector 1 -0.5 0)
                              (cl-transforms:make-quaternion 0 0 1 1)))

(defparameter *shelf-pose* (cl-transforms-stamped:make-pose-stamped
                            "map" 0
                            (cl-transforms:make-3d-vector 1 1 1.3)
                            (cl-transforms:make-identity-rotation)))

(defun grasp-object-from-shelf (?object)
  (let ((?table *table*)
        (?search-pose *pose-searching*)
        (?newobject (desig:an object (type ?object)))
        (?pose-placing *placing-pose*)
        ?dropping-pose)
    

    (exe:perform
     (desig:an action
               (type going)
               (target (desig:a location
                                (pose ?pose-placing)))))

    (desig:an action 
               (type looking)
               (target (desig:a location (pose (cl-transforms-stamped:make-pose-stamped 
                                                "map" 0
                                                (cl-transforms:make-3d-vector 1 0.63 0.7)
                                                (cl-transforms:make-quaternion))))))
    (setf ?newobject
          (exe:perform
           (desig:a motion
                    (type detecting)
                    (object ?newobject))))
    
    (exe:perform
     (desig:an action
               (type positioning-arm)
               (left-joint-states ((l_shoulder_pan_joint 1.2)))
               (right-configuration park)))

    (exe:perform
     (desig:an action
               (type fetching)
               (object ?newobject)
               (arm :right)
               (location (desig:a location
                                  (pose ?search-pose)))))

    (setf (btr:joint-state (btr:get-robot-object) "l_shoulder_pan_joint") 0.7)
    (setf (btr:joint-state (btr:get-robot-object) "l_shoulder_lift_joint") 0.5)
    (setf (btr:joint-state (btr:get-robot-object) "l_upper_arm_roll_joint") 1.4)
    (setf (btr:joint-state (btr:get-robot-object) "l_elbow_flex_joint") -1.5)
    (setf (btr:joint-state (btr:get-robot-object) "l_forearm_roll_joint") -1.15)
    (setf (btr:joint-state (btr:get-robot-object) "l_wrist_flex_joint") 0)

          
    (exe:perform
     (desig:an action
               (type going)
               (target (desig:a location
                                (pose ?pose-placing)))))
    

    (let* ((map->basket (cl-transforms:pose->transform
                         (btr:pose (btr:object btr:*current-bullet-world* :b))))
           (basket->object (cl-transforms:make-transform
                            (cl-transforms:make-3d-vector 0.15 -0.15 0.25)
                            (cl-transforms:make-quaternion 0 0 1 0)))
           (map->object (cl-transforms:transform* map->basket basket->object))
           (?dropping-pose (cl-transforms-stamped:pose->pose-stamped
                           "map" 0
                           (cl-transforms:transform->pose map->object))))

      (btr:add-vis-axis-object ?dropping-pose)

      
      ;; (exe:perform 
      ;;  (desig:a motion
      ;;           (type moving-tcp)
      ;;           (right-pose ?dropping-pose)
      ;;           (collision-mode :avoid-all)))

      ;; (exe:perform
      ;;  (desig:an action
      ;;            (type setting-gripper)
      ;;            (gripper :right)
      ;;            (position 0.1)))

      
      ;; (btr:detach-object (btr:get-robot-object)
      ;;                    (btr:object btr:*current-bullet-world* (desig:desig-prop-value ?newobject 'name)))

      ;; (btr:attach-object (btr:object btr:*current-bullet-world* ?object)
      ;;                    (btr:object btr:*current-bullet-world* :b))

      ;;(btr:simulate btr:*current-bullet-world* 1)
      
      (exe:perform
       (desig:an action
                 (type placing)
                 (object ?newobject)
                 (arm :right)
                 (target (desig:a location (pose ?dropping-pose)))))
      
      )))

(defun place-object-in-shelf (?object-type ?destination)
  "?object-type is a variable of type symbol and ?destination is a variable of type pose-stamped"
  (let ((?object (desig:an object (type ?object-type)))
        (?table-pose *table*)
        (?shelf-pose *shelf-pose*))

    (exe:perform
     (desig:an action
               (type transporting)
               ;;(arm :left)
               (object ?object)
               (location (desig:a location
                                  (pose ?table-pose)))
               (target (desig:a location
                                (pose ?destination)))))

    
  ))


(defun demo ()
  (urdf-proj:with-simulated-robot
    (place-object-in-shelf :denkmit (cl-transforms-stamped:make-pose-stamped
                                     "map" 0
                                     (cl-transforms:make-3d-vector -1 1 1.2)
                                     (cl-transforms:make-identity-rotation)))

    (place-object-in-shelf :heitmann (cl-transforms-stamped:make-pose-stamped
                                      "map" 0
                                      (cl-transforms:make-3d-vector -1.2 1 1.2)
                                      (cl-transforms:make-identity-rotation)))
    (grasp-object-from-shelf :dove)
    (grasp-object-from-shelf :denkmit)
  
  ))
