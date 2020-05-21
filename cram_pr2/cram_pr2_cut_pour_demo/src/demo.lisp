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

(defparameter *weisswurst-detect* nil)


(defun clean-demo ()
  (btr:detach-all-objects (btr:get-robot-object))
  (btr-utils:kill-all-objects)
  (sleep 0.3))

(defun park ()
   (exe:perform (desig:an action
                         (type positioning-arm)
                         (left-configuration park)
                         (right-configuration park))))


  ;;###############################################################################
  ;;                                    SPAWNING
  ;;###############################################################################


(defun spawn-weisswurst(location)
  (if (eq location :sink)
      (btr-utils:spawn-object 'weisswurst-1 :weisswurst
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector 1.4 0.6 0.9)
                                     (cl-tf:make-quaternion 0 0 1 1)))
      (btr-utils:spawn-object 'weisswurst-1 :weisswurst
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector -0.95 0.9 0.9)
                                     (cl-tf:make-quaternion 0 0 -1 1)))))
(defun spawn-pr2(location)
  (if (eq location :sink)
      (prolog:prolog '(and (btr:bullet-world ?world)
                       (assert (btr:object-pose ?world cram-pr2-description:pr2
                                ((0.73 0.6 0) (0 0 0 1))))))
      (prolog:prolog '(and (btr:bullet-world ?world)
                       (assert (btr:object-pose ?world cram-pr2-description:pr2
                                ((-0.3 0.9 0) (0 0 1 0))))))))

(defun spawn-bread (location)
  (if (eq location :sink)
      (btr-utils:spawn-object 'bread-1 :bread
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector 1.4 0.6 0.95)
                                     (cl-tf:make-quaternion 0 0 1 1)))

      (btr-utils:spawn-object 'bread-1 :bread
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector -0.85 0.9 0.95)
                                     (cl-tf:make-quaternion 0 0 -1 1)))))

(defun spawn-knife (location)
  (if (eq location :sink)
      (btr-utils:spawn-object 'knife-1 :knife
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector 1.5 0.6 1.15)
                                     (cl-tf:make-quaternion 1 -1 1 1)))
      (btr-utils:spawn-object 'knife-1 :knife
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector -0.95 0.9 1.15)
                                     (cl-tf:make-quaternion 1 1 1 -1)))))

(defun spawn-big-knife (location)
  (if (eq location :sink)
      (btr-utils:spawn-object 'big-knife-1 :big-knife
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector 1.5 0.6 1.4)
                                     (cl-tf:make-quaternion -1 -1 1 1)))
      (btr-utils:spawn-object 'big-knife-1 :big-knife
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector -0.95 1.2 1.3)
                                     (cl-tf:make-quaternion 1 -1 1 -1)))))

(defun spawn-bottle (location)
  (if (eq location :sink)
      (btr-utils:spawn-object 'bottle-1 :bottle
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector 1.4 0.8 0.95)
                                     (cl-tf:make-identity-rotation)))
      (btr-utils:spawn-object 'bottle-1 :bottle
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector -0.85 1.1 0.95)
                                     (cl-tf:make-identity-rotation)))))
      
    
(defun spawn-cup (location)
  (if (eq location :sink)
      (btr-utils:spawn-object 'cup-1 :cup
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector 1.4 0.4 0.95)
                                     (cl-tf:make-identity-rotation)))
      (btr-utils:spawn-object 'cup-1 :cup
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector -0.85 0.6 0.95)
                                     (cl-tf:make-identity-rotation)))))

(defun spawn-cup-to-pour-into (location)
  (if (eq location :sink)
      (btr-utils:spawn-object 'cup-2 :cup
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector 1.4 0.6 0.95)
                                     (cl-tf:make-quaternion 0 0 0 1)))

      (btr-utils:spawn-object 'cup-2 :cup
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector -0.85 0.9 0.95)
                                     (cl-tf:make-quaternion 0 0 0 1)))))



  ;;###############################################################################
  ;;                                    SLICING
  ;;###############################################################################


(defun demo-play-slice (bread-or-weisswurst island-or-sink)
  (clean-demo)
  (when (and (or (equal bread-or-weisswurst :bread)
                 (equal bread-or-weisswurst :weisswurst))
             (or (equal island-or-sink :island)
                 (equal island-or-sink :sink)))
    (let ((?knife (if (equal bread-or-weisswurst :bread)
                      :big-knife
                      :knife)))
      (urdf-proj:with-simulated-robot
        (park)
        (demo-slice :right bread-or-weisswurst
                    ?knife island-or-sink
                    :allow-hand))
      (clean-demo)
      (urdf-proj:with-simulated-robot
        (park)
        (demo-slice :left bread-or-weisswurst
                    ?knife island-or-sink
                    :allow-hand)))))



(defun demo-slice (?arm-to-slice ?object-type ?knife-type ?location ?arms-collision-mode)
  (when (not (or (and (eq ?object-type :weisswurst) (eq ?knife-type :knife))
                 (and (eq ?object-type :bread) (eq ?knife-type :big-knife))))
    (error "Types are not compatible: ~a is not sliceable with ~a~%."
           ?object-type ?knife-type))
  (let ((?object-to-slice nil)
        (?object-knife nil)
        (?grasp-to-hold nil)
        (?slice-position nil)
        (?arm-to-hold nil)
        (?pose-slice-object nil)
        (?pose-knife nil)
        (?collision-mode ?arms-collision-mode))
    
    (if (eq ?object-type :weisswurst)
        (progn
          (spawn-weisswurst ?location)
          (spawn-knife ?location)
          (setf ?pose-knife (cl-tf:pose->pose-stamped
                             cram-tf:*fixed-frame*
                             0 
                             (btr:object-pose 'knife-1)))
          (setf ?pose-slice-object (cl-tf:pose->pose-stamped
                                    cram-tf:*fixed-frame*
                                    0
                                    (btr:object-pose 'weisswurst-1))))
        (progn
          (spawn-bread ?location)
          (spawn-big-knife ?location)
          (setf ?pose-knife (cl-tf:pose->pose-stamped
                             cram-tf:*fixed-frame*
                             0
                             (btr:object-pose 'big-knife-1)))
          (setf ?pose-slice-object (cl-tf:pose->pose-stamped
                                    cram-tf:*fixed-frame*
                                    0
                                    (btr:object-pose 'bread-1)))))

    
    (spawn-pr2 ?location)
    
;;;;; Looking at the knife

    (cpl:with-retry-counters ((looking-retry 3))
      (cpl:with-failure-handling
          ((common-fail:low-level-failure
               (e)
             (declare (ignore e))
             (cpl:do-retry looking-retry
               (roslisp:ros-warn (slice-demo looking-fail)
                                 "~%Failed to look at given position~%")
               (cpl:retry))
             (roslisp:ros-warn (slice-demo looking-fail)
                               "~%No more retries~%")))
        (dotimes (n 3)
          (cram-executive:perform
           (desig:an action
                     (type looking)
                     (target (desig:a location (pose ?pose-knife))))))))
    
;;;;; Detecting the knife

    (cpl:with-retry-counters ((detecting-retry 5))
      (cpl:with-failure-handling
          ((common-fail:high-level-failure (e)
             (declare (ignore e))
             (cpl:do-retry detecting-retry
               (roslisp:ros-warn (slice-demo detecting-fail)
                                 "~%Failed to detect ~a~%" ?knife-type)
               (cpl:retry))
             (roslisp:ros-warn (slice-demo detecting file)
                               "~%No more retries~%")))
        (setf ?object-knife
              (urdf-proj::detect (desig:an object (type ?knife-type))))))
    
;;;;; Setting parameters
    
    (if (eq ?arm-to-slice :right)
        (progn
          (setf ?grasp-to-hold :left-hold)
          (setf ?slice-position :right-top)
          (setf ?arm-to-hold :left))
        (progn
          (setf ?grasp-to-hold :right-hold)
          (setf ?slice-position :left-top)
          (setf ?arm-to-hold :right)))

    
;;;;; Picking the knife up
    
    (exe:perform (desig:an action
                           (type picking-up)
                           (object ?object-knife)
                           (arm ?arm-to-slice) 
                           (grasp front)))

    
;;;;; Parking arm
    
    (if (eq ?arm-to-slice :right)
        (exe:perform (desig:an action
                               (type positioning-arm)
                               (right-configuration park)))
        
        (exe:perform (desig:an action
                               (type positioning-arm)
                               (left-configuration park))))

    
;;;;; Looking at object to slice

    (cpl:with-retry-counters ((looking-retry 3))
      (cpl:with-failure-handling
          ((common-fail:low-level-failure
               (e)
             (declare (ignore e))
             (cpl:do-retry looking-retry
               (roslisp:ros-warn (slice-demo looking-fail)
                                 "~%Failed to look at given position~%")
               (cpl:retry))
             (roslisp:ros-warn (slice-demo looking-fail)
                               "~%No more retries~%")))
        (dotimes (n 3)
          (cram-executive:perform
           (desig:an action
                     (type looking)
                     (target (desig:a location (pose ?pose-slice-object))))))))
    
;;;;; Detecting the object to slice

    (cpl:with-retry-counters ((detecting-retry 3))
      (cpl:with-failure-handling
          ((common-fail:high-level-failure (e)
             (declare (ignore e))
             (cpl:do-retry detecting-retry
               (roslisp:ros-warn (slice-demo detecting-fail)
                                 "~%Failed to detect ~a~%" ?knife-type)
               (cpl:retry))
             (roslisp:ros-warn (slice-demo detecting-fail)
                               "~%No more retries~%")))
        (setf ?object-to-slice
              (urdf-proj::detect (desig:an object (type ?object-type))))))  

;;;;; Holding the object to slice with the other hand
    
    (cpl:with-retry-counters ((holding-retry 3))
      (cpl:with-failure-handling
          ((common-fail:high-level-failure
               (e)
             (declare (ignore e))
             (cpl:do-retry holding-retry
               (roslisp:ros-warn (slice-demo holding-fail)
                                 "~%Failed to hold ~a~%" ?object-type)
               (cpl:retry))
             (roslisp:ros-warn (slice-demo holding-fail)
                               "~%No more retries~%")))
        
        (exe:perform (desig:an action
                               (type holding)
                               (object ?object-to-slice)
                               (arm ?arm-to-hold) 
                               (grasp ?grasp-to-hold)))))

    
;;;;; Slicing the object

    (cpl:with-retry-counters ((slicing-retry 3))
      (cpl:with-failure-handling
          ((common-fail:high-level-failure
               (e)
             (declare (ignore e))
             (cpl:do-retry slicing-retry
               (roslisp:ros-warn (slice-demo slicing-fail)
                                 "~%Failed to slice ~a~%" ?object-type)
               (cpl:retry))
             (roslisp:ros-warn (slice-demo slicing-fail)
                               "~%No more retries~%")))
        
        
        (exe:perform (desig:an action
                               (type slicing)
                               (object ?object-to-slice)
                               (arm ?arm-to-slice) 
                               (grasp ?slice-position)
                               (desig:when ?collision-mode
                                 (collision-mode ?collision-mode))))))))



  ;;###############################################################################
  ;;                                    POURING
  ;;###############################################################################

(defun demo-play-pour (island-or-sink)
  (when (or (equal island-or-sink :sink)
            (equal island-or-sink :island))
    (demo-pour island-or-sink)))

(defun demo-pour (island-or-sink)
  (urdf-proj:with-simulated-robot
    (clean-demo)
    (park)
    (spawn-pr2 island-or-sink)
    (spawn-cup island-or-sink)
    (spawn-bottle island-or-sink)

    (let* ((?pose-cup-1 (cl-tf:pose->pose-stamped
                         cram-tf:*fixed-frame*
                         0
                         (btr:object-pose 'cup-1)))
           (?object-cup-1 nil)
           (?pose-bottle-1 (cl-tf:pose->pose-stamped
                            cram-tf:*fixed-frame*
                            0
                            (btr:object-pose 'bottle-1)))
           (?object-bottle-1 nil)
           (?object-cup-2 nil))
      
      
;;;;; Looking at object to pour from

      (cpl:with-retry-counters ((looking-retry 3))
        (cpl:with-failure-handling
            ((common-fail:low-level-failure
                 (e)
               (declare (ignore e))
               (cpl:do-retry looking-retry
                 (roslisp:ros-warn (slice-demo looking-fail)
                                   "~%Failed to look at given position~%")
                 (cpl:retry))
               (roslisp:ros-warn (slice-demo looking-fail)
                                 "~%No more retries~%")))
          (dotimes (n 3)
            (cram-executive:perform
             (desig:an action
                       (type looking)
                       (target (desig:a location (pose ?pose-cup-1))))))))
      
;;;;; Detecting the object to pour from

      (cpl:with-retry-counters ((detecting-retry 3))
        (cpl:with-failure-handling
            ((common-fail:high-level-failure (e)
               (declare (ignore e))
               (cpl:do-retry detecting-retry
                 (roslisp:ros-warn (slice-demo detecting-fail)
                                   "~%Failed to detect bottle")
                 (cpl:retry))
               (roslisp:ros-warn (slice-demo detecting-fail)
                                 "~%No more retries~%")))
          (setf ?object-cup-1
                (urdf-proj::detect (desig:an object (type :cup))))))

;;;;; Picking-up the object to pour from

      (let ((?arm :right)
            (?possible-arms '(:right :left)))
        (cpl:with-retry-counters ((arm-change-retry 1))
          (cpl:with-failure-handling
              ((common-fail:manipulation-pose-unreachable (e)
                 (roslisp:ros-warn (arm-failure) "Manipulation failed: ~a~%" e)
                 (cpl:do-retry arm-change-retry
                   (setf ?arm (car (remove ?arm ?possible-arms)))
                   (cpl:retry))
                 (roslisp:ros-warn (arm-failures) "No more retries left")))
 
            (exe:perform (desig:an action
                                   (type picking-up)
                                   (object ?object-cup-1)
                                   (arm ?arm) 
                                   (grasp right-side))))))

;;;;; Looking at other object to pour from

      (cpl:with-retry-counters ((looking-retry 3))
        (cpl:with-failure-handling
            ((common-fail:low-level-failure
                 (e)
               (declare (ignore e))
               (cpl:do-retry looking-retry
                 (roslisp:ros-warn (slice-demo looking-fail)
                                   "~%Failed to look at given position~%")
                 (cpl:retry))
               (roslisp:ros-warn (slice-demo looking-fail)
                                 "~%No more retries~%")))
          (dotimes (n 3)
            (cram-executive:perform
             (desig:an action
                       (type looking)
                       (target (desig:a location (pose ?pose-bottle-1))))))))
      
;;;;; Detecting the other object to pour from

      (cpl:with-retry-counters ((detecting-retry 3))
        (cpl:with-failure-handling
            ((common-fail:high-level-failure (e)
               (declare (ignore e))
               (cpl:do-retry detecting-retry
                 (roslisp:ros-warn (slice-demo detecting-fail)
                                   "~%Failed to detect bottle")
                 (cpl:retry))
               (roslisp:ros-warn (slice-demo detecting-fail)
                                 "~%No more retries~%")))
          (setf ?object-bottle-1
                (urdf-proj::detect (desig:an object (type :bottle))))))

;;;;; Picking up the other object to pour from
      
      (let ((?arm :right)
            (?possible-arms '(:right :left)))
        (cpl:with-retry-counters ((arm-change-retry 1))
          (cpl:with-failure-handling
              ((common-fail:manipulation-pose-unreachable (e)
                 (roslisp:ros-warn (arm-failure) "Manipulation failed: ~a~%" e)
                 (cpl:do-retry arm-change-retry
                   (setf ?arm (car (remove ?arm ?possible-arms)))
                   (cpl:retry))
                 (roslisp:ros-warn (arm-failures) "No more retries left")))
            
            (exe:perform (desig:an action
                                   (type picking-up)
                                   (object ?object-bottle-1)
                                   (arm ?arm) 
                                   (grasp left-side))))))

;;;;; Park the robot and spawn the object to pour into
      
      (park)
      (spawn-cup-to-pour-into island-or-sink)

;;;;; Pouring into the object spawned
      
      (cpl:with-retry-counters ((detecting-retry 3))
        (cpl:with-failure-handling
            ((common-fail:high-level-failure (e)
               (declare (ignore e))
               (cpl:do-retry detecting-retry
                 (roslisp:ros-warn (slice-demo detecting-fail)
                                   "~%Failed to detect bottle")
                 (cpl:retry))
               (roslisp:ros-warn (slice-demo detecting-fail)
                                 "~%No more retries~%")))
          (setf ?object-cup-2
                (urdf-proj::detect (desig:an object (type :cup))))))

;;;;; Pouring from the left-side of the spawned object with the
;;;;; object holded in the left arm
      
      (cram-executive:perform
              (desig:an action
                        (type pouring)
                        (object ?object-cup-2)
                        (arms (left))
                        (grasp left-side)))

;;;;; Parking the robot
      (sleep 2.0)
      (park)


;;;;; Pouring from the right-side of the spawned object with the
;;;;; object holded in the right arm
      (cram-executive:perform
       (desig:an action
                 (type pouring)
                 (object ?object-cup-2)
                 (arms (right))
                 (grasp right-side)))

;;;;; Parking the robot
      (sleep 2.0)
      (park)

;;;;; Pouring from the back of the spawned object with the
;;;;; object holded in the right arm
      (cram-executive:perform
              (desig:an action
                        (type pouring)
                        (object ?object-cup-2)
                        (arms (right))
                        (grasp back)))
      
 ;;;;; Parking the robot
      (sleep 2.0)
      (park)

;;;;; Pouring from the back of the spawned object with the
;;;;; object holded in the left arm
      (cram-executive:perform
       (desig:an action
                 (type pouring)
                 (object ?object-cup-2)
                 (arms (left))
                 (grasp back))))))
