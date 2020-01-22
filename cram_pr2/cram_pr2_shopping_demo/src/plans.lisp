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
                       (cl-transforms:make-3d-vector -3.5 0.1 0.7)
                       (cl-transforms:make-identity-rotation)))

(defparameter *pose-searching-2* (cl-transforms-stamped:make-pose-stamped
                                "map" 0
                                (cl-transforms:make-3d-vector 1 0.7 1)
                                (cl-transforms:make-identity-rotation)))

(defparameter *placing-pose-2* (cl-transforms-stamped:make-pose-stamped
                              "map" 0
                              (cl-transforms:make-3d-vector 1 -0.5 0)
                              (cl-transforms:make-quaternion 0 0 1 1)))

(defparameter *pose-searching-1* (cl-transforms-stamped:make-pose-stamped
                                "map" 0
                                (cl-transforms:make-3d-vector -1 0.7 1)
                                (cl-transforms:make-identity-rotation)))

(defparameter *placing-pose-1* (cl-transforms-stamped:make-pose-stamped
                              "map" 0
                              (cl-transforms:make-3d-vector -1 -0.5 0)
                              (cl-transforms:make-quaternion 0 0 1 1)))

(defparameter *shelf-pose* (cl-transforms-stamped:make-pose-stamped
                            "map" 0
                            (cl-transforms:make-3d-vector 1 1 1.3)
                            (cl-transforms:make-identity-rotation)))

(defun grasp-object-from-shelf (?object shelf) 
  (let ((?newobject (desig:an object (type ?object)))
        (?search-pose (if (eql shelf 1) *pose-searching-1* *pose-searching-2*))
        (?pose-placing (if (eql shelf 1) *placing-pose-2* *placing-pose-2*))
        (?look-pose (if (eql shelf 1) (cl-transforms-stamped:make-pose-stamped 
                                                "map" 0
                                                (cl-transforms:make-3d-vector -1 0.63 0.7)
                                                (cl-transforms:make-quaternion 0 0 0 1))
                        (cl-transforms-stamped:make-pose-stamped 
                                                "map" 0
                                                (cl-transforms:make-3d-vector 1 0.63 0.7)
                                                (cl-transforms:make-quaternion 0 0 0 1)))))

     
    (exe:perform
     (desig:an action
               (type positioning-arm)
               (left-configuration park)
               (right-configuration park)))

    (exe:perform
     (desig:an action
               (type going)
               (target (desig:a location
                                (pose ?pose-placing)))))
    (exe:perform
     (desig:an action 
               (type looking)
               (target (desig:a location (pose ?look-pose)))))
    
    (setf ?newobject
          (exe:perform
           (desig:a motion
                    (type detecting)
                    (object ?newobject))))

    (exe:perform
     (desig:an action
               (type fetching)
               (object ?newobject)
               (arm :right)
               (location (desig:a location
                                  (pose ?search-pose)))))
    (exe:perform 
     (desig:a motion
              (type moving-arm-joints)
              (left-joint-states (("l_shoulder_pan_joint" 0.7)
                                  ("l_shoulder_lift_joint" 0.5)
                                  ("l_upper_arm_roll_joint" 1.4)
                                  ("l_elbow_flex_joint" -1.5)
                                  ("l_forearm_roll_joint" -1.15)
                                  ("l_wrist_flex_joint" 0)))))
                                 

          
    (exe:perform
     (desig:an action
               (type going)
               (target (desig:a location
                                (pose ?pose-placing)))))
    

    (let* ((map->basket (cl-transforms:pose->transform
                         (btr:pose (btr:object btr:*current-bullet-world* :b))))
           (basket->object (cl-transforms:make-transform
                            (cl-transforms:make-3d-vector 0.15 -0.1 0.05)
                            (cl-transforms:make-quaternion 0 0 1 0)))
           (map->object (cl-transforms:transform*  map->basket basket->object))
           (?dropping-pose (cl-transforms-stamped:pose->pose-stamped
                           "map" 0
                           (cl-transforms:transform->pose map->object)))
           (?basket-desig (desig:an object (type :basket)))
           (basket-pose (btr:pose (btr:object btr:*current-bullet-world* :b)))
           (map->basket (cl-transforms:pose->transform basket-pose))
           (map->base (cl-transforms:pose->transform (btr:pose (btr:get-robot-object))))
           (base->basket (cl-transforms:transform*
                          (cl-transforms:transform-inv map->base)
                          map->basket)))

      (setf ?basket-desig
            (desig:copy-designator ?basket-desig
                                   :new-description
                                   `((:type :basket)
                                     (:name :b)
                                     (:pose ((:pose ,(cl-transforms-stamped:pose->pose-stamped
                                                      "base_footprint" 0
                                                      (cl-transforms:transform->pose base->basket)))
                                             (:transform ,(cl-transforms-stamped:transform->transform-stamped
                                                           "base_footprint" "b" 0
                                                           base->basket))
                                             (:pose-in-map ,(cl-transforms-stamped:pose->pose-stamped
                                                             "map" 0
                                                             basket-pose))
                                             (:transform-in-map ,(cl-transforms-stamped:transform->transform-stamped
                                                                  "map" "b" 0
                                                                  (cl-transforms:pose->transform basket-pose))))))))

      
      
      (exe:perform
       (desig:an action
                 (type placing)
                 (object ?newobject)
                 (arm right)
                 ;;(target (desig:a location (pose ?dropping-pose)))))
                 (target (desig:a location (on ?basket-desig) (for ?newobject) (attachment asd)))))
      
      )))

(defun place-object-in-shelf (?object-type ?destination &optional pose-2 pose-3 pose-4)
  "?object-type is a variable of type symbol and ?destination is a variable of type pose-stamped"
  (let ((?object (desig:an object (type ?object-type)))
        (?table *table*)
        (?target-poses `(,?destination ,pose-2 ,pose-3 ,pose-4)))
    
    (exe:perform
     (desig:an action
               (type transporting)
               (arm right)
               (object ?object)
               (location (desig:a location
                                  (pose ?table)))
               (target (desig:a location
                                (poses  ?target-poses)))))

    
  ))


(defun demo ()
  (urdf-proj:with-simulated-robot
    (place-object-in-shelf :denkmit (cl-transforms-stamped:make-pose-stamped
                                     "map" 0
                                     (cl-transforms:make-3d-vector 0.7 0.7 0.68)
                                     (cl-transforms:make-quaternion 0 0 1 0))
                           (cl-transforms-stamped:make-pose-stamped
                            "map" 0
                            (cl-transforms:make-3d-vector 0.7 0.7 0.68)
                            (cl-transforms:make-quaternion 0 0 1 1))
                           (cl-transforms-stamped:make-pose-stamped
                            "map" 0
                            (cl-transforms:make-3d-vector 0.7 0.7 0.68)
                            (cl-transforms:make-quaternion 0 0 -1 0))
                           (cl-transforms-stamped:make-pose-stamped
                            "map" 0
                            (cl-transforms:make-3d-vector 0.7 0.7 0.68)
                            (cl-transforms:make-quaternion 0 0 -1 1)))
    
    
    (grasp-object-from-shelf :dove 1)
    (grasp-object-from-shelf :heitmann 2)
  
  ))
