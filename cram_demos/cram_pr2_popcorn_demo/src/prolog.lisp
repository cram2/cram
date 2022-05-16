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

(defun get-pose (?frame)
  (cram-tf:transform->pose-stamped 
   cram-tf:*fixed-frame*
   0.0
   (cram-tf:apply-transform 
    (cl-tf:lookup-transform cram-tf:*transformer* cram-tf:*fixed-frame* ?frame)
    (cl-tf:transform->transform-stamped 
     ?frame
     ?frame
     0.0
     (cl-tf:pose->transform
      (cl-tf:pose-stamped->pose
       (get-rotate-pose-in-knob-frame 0 ?frame)))))))
    
(defun get-rotate-pose-in-knob-frame (z-rotation 
                                      &optional (knob-frame "iai_popcorn_stove_knob_1"))
             
  (cl-tf:make-pose-stamped
   knob-frame
   0.0
   (cl-tf:make-3d-vector 0.0 0.0 0.034)
   (cl-tf:euler->quaternion :ax (/ 3.14 2) 
                            :ay (/ 3.14 2)
                            :az z-rotation)))

(defun turn-knob (&key
                    ((:arm ?arm))
                    ((:frame ?knob-frame))
                    ((:pose ?knob-pose))
                    ((:configuration ?on-or-off))
                  &allow-other-keys)
  (declare (keyword ?on-or-off)
           (string ?knob-frame))
  
  ;; Look at knob
  (roslisp:ros-info (popcorn turn-knob) "Looking at knob")
  (exe:perform (desig:a motion
                        (type looking)
                        (pose ?knob-pose)))

  (roslisp:ros-info (popcorn turn-knob) "Opening gripper to grasp knob")
  ;; Opening the gripper
  (exe:perform
   (desig:an action
             (type setting-gripper)
             (gripper ?arm)
             (position 0.1)))

  (roslisp:ros-info (popcorn turn-knob) "Moving arm to knob")
  ;; Moving arm to knob
  (exe:perform
   (desig:a motion
            (type moving-tcp)
            (desig:when (eq ?arm :right)
              (right-pose ?knob-pose))
            (desig:when (eq ?arm :left)
              (left-pose ?knob-pose))
            (collision-mode :allow-hand)))
  
  (roslisp:ros-info (popcorn turn-knob) "Closing gripper to turn knob")
  ;; Closing the gripper
  (exe:perform
   (desig:an action
             (type setting-gripper)
             (gripper ?arm)
             (position 0.018)))

 
  ;; Turn knob
  (roslisp:ros-info (popcorn turn-knob) "Turning knob")
  (let* ((offset 10)
         (offset-rotation (cond
                            ((eq ?on-or-off :on) offset)
                            ((eq ?on-or-off :off) (* -1 offset))
                            (t 10)))
         (knob-joint (concatenate 'string ?knob-frame "_joint"))
         (?knob-rotate-pose-in-knob-frame (get-rotate-pose-in-knob-frame
                                             (* -1 
                                                (cram-math:degrees->radians
                                                 offset-rotation))
                                             ?knob-frame)))
      (loop with ?knob-rotate-pose = nil
            with start = 0
            with end = 90
            for degree from start to end by offset do
              ;; Calculate the pose of the knob
              (setf ?knob-rotate-pose
                    (make-pose-absolute (cons ?knob-frame
                                        ?knob-rotate-pose-in-knob-frame)))
              ;; Moving the robots arm
              (exe:perform
               (desig:a motion
                        (type moving-tcp)
                        (desig:when (eq ?arm :right)
                          (right-pose ?knob-rotate-pose))
                        (desig:when (eq ?arm :left)
                          (left-pose ?knob-rotate-pose))
                        (collision-mode :allow-hand)))
              ;; Setting the joint state of the knob accordingly
              (let ((joint-state (cond
                                   ((eq ?on-or-off :on) degree)
                                   ((eq ?on-or-off :off) (- 90 degree))
                                   (t degree))))
                (setf (btr:joint-state (btr:get-environment-object)
                                       knob-joint)
                      (* -1 
                         (cram-math:degrees->radians joint-state))))))
  (roslisp:ros-info (popcorn turn-knob) "Turned knob")

  ;; Park arms
  (roslisp:ros-info (popcorn turn-knob) "Parking arms")
  (exe:perform
   (desig:an action
             (type positioning-arm)
             (left-configuration park)
             (right-configuration park))))

(defun salt (&key
               ((:object ?salt))
               ((:on-object ?other-object))
             &allow-other-keys)
  (let* ((salt-frame (desig:desig-prop-value ?salt :name))
         (other-object-frame (desig:desig-prop-value ?other-object :name))
         (other-object-type (desig:desig-prop-value ?other-object :type))
         (right-hand-position (cl-tf:translation
                               (man-int:get-object-type-robot-frame-tilt-approach-transform
                                other-object-type :right :right-side)))
         (left-hand-position (cl-tf:translation
                              (man-int:get-object-type-robot-frame-tilt-approach-transform
                               other-object-type :left :left-side)))
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
          other-object-frame
          0.0
          dynamic-right-hand-position 
          (cl-tf:euler->quaternion :ax pi :ay 0.0 :az 0))
         :?left-arm-pose
         (cl-tf:make-pose-stamped
          other-object-frame
          0.0
          dynamic-left-hand-position
          (cl-tf:euler->quaternion :ax pi :ay 0.0 :az pi)))
        
        (sleep 0.5)
        
        ;; Salting the pocorn by ....
        ;; ... rotating both endeffectors towards the robot
        (dotimes (angle-c 5)
          (sleep 0.1)
          (move-arms 
           :?right-arm-pose
           (cl-tf:make-pose-stamped
            salt-frame
            0.0
            (cl-tf:make-3d-vector 0 0 0.035)
            (cl-tf:euler->quaternion :ax 0.0 :ay 0.0 :az
                                     (+ (/ pi 2)
                                        (* 2
                                           (*
                                            angle-c
                                            (/ pi 36))))))
           :?left-arm-pose
           (cl-tf:make-pose-stamped
            salt-frame
            0.0
            (cl-tf:make-identity-vector)
            (cl-tf:euler->quaternion :ax 0.0 :ay 0.0 :az 
                                     (- 
                                      (+ pi (/ pi 2))
                                      (*
                                       angle-c
                                       (/ pi 36)))))))
        
        (sleep 0.5)
        ;; ... and rotating back.
        (sleep 0.5)
        ;; repeat for another pose above the plate
        ))))

(def-fact-group popcorn-actions (desig:action-grounding)
 
  ;;type turning-knob
  (<- (desig:action-grounding ?action-designator (turn-knob ?resolved-action-designator))
    (spec:property ?action-designator (:type :turning-knob))

    (spec:property ?action-designator (:arm ?arm))
    (man-int:robot-free-hand ?_ ?arm)
    
    (spec:property ?action-designator (:frame ?knob-frame))

    (lisp-fun get-pose ?knob-frame ?knob-pose)
    (not (equal ?knob-pose nil))

    (spec:property ?action-designator (:configuration ?on-or-off))
    (member ?on-or-off (:on :off))
    
    (desig:designator :action ((:type :turning-knob)
                               (:arm ?arm)
                               (:frame ?knob-frame)
                               (:pose ?knob-pose)
                               (:configuration ?on-or-off))
                      ?resolved-action-designator))

  ;;type salting
  (<- (desig:action-grounding ?action-designator (salt ?resolved-action-designator))
    (spec:property ?action-designator (:type :salting))

    (spec:property ?action-designator (:object ?object-designator))
    (spec:property ?object-designator (:type ?object-type))
    (spec:property ?object-designator (:name ?object-frame))
    
    (spec:property ?action-designator (:on-object ?other-object-designator))
    (spec:property ?other-object-designator (:type ?other-object-type))
    (spec:property ?other-object-designator (:name ?other-object-name))
    (desig:designator :action ((:type :salting)
                               (:object ?object-designator)
                               (:on-object ?other-object-designator))
                      ?resolved-action-designator)))
