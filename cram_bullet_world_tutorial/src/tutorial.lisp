;;;
;;; Copyright (c) 2017, Arthur Niedzwiecki <niedzwiecki@uni-bremen.de>
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

(in-package :btw-tut)

(defun get-kitchen-urdf ()
  (slot-value
   (btr:object btr:*current-bullet-world* :kitchen)
   'cram-bullet-reasoning:urdf))

(defun move-kitchen-joint (&key (joint-name "iai_fridge_door_joint")
                             (joint-angle 0.2d0) (kitchen-name :kitchen))
  (btr:set-robot-state-from-joints
   `((,joint-name  ,joint-angle))
   (btr:object btr:*current-bullet-world* kitchen-name)))

(defun add-objects-to-mesh-list (&optional (ros-package "cram_bullet_world_tutorial"))
  (mapcar (lambda (object-filename-and-object-extension)
            (declare (type list object-filename-and-object-extension))
            (destructuring-bind (object-filename object-extension)
                object-filename-and-object-extension
              (let ((lisp-name (roslisp-utilities:lispify-ros-underscore-name
                                object-filename :keyword)))
                (pushnew (list lisp-name
                               (format nil "package://~a/resource/~a.~a"
                                       ros-package object-filename object-extension)
                               nil)
                         btr::*mesh-files*
                         :key #'car)
                lisp-name)))
          (mapcar (lambda (pathname)
                    (list (pathname-name pathname) (pathname-type pathname)))
                  (directory (physics-utils:parse-uri
                              (format nil "package://~a/resource/*.*" ros-package))))))

(defparameter *pose-meal-table*
  (cl-tf:make-pose-stamped
   "map" 0.0
   (cl-tf:make-3d-vector -0.15 2.0 0)
   (cl-tf:make-quaternion 0.0d0 0.0d0 -1.0d0 0.0d0)))

(defparameter *pose-bottle-1*
  (cl-transforms-stamped:make-pose-stamped 
   "map" 0.0 
   (cl-transforms:make-3d-vector -2 -0.9d0 0.861667d0)
   (cl-tf:make-quaternion 0.0d0 0.0d0 -1.0d0 0.0d0)))

(defparameter *pose-bottle-2*
  (cl-transforms-stamped:make-pose-stamped 
   "map" 0.0 
   (cl-transforms:make-3d-vector -0.7 1.8 1.0d0)
   (cl-transforms:make-identity-rotation)))

(defparameter *pose-counter*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -2.1547d0 -0.381d0 0.0d0)
   (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi -2))))

(defun spawn-two-bottles ()
  (unless (assoc :bottle btr::*mesh-files*)
    (add-objects-to-mesh-list))
  (prolog:prolog '(and (btr:bullet-world ?world)
              (assert (btr:object ?world :mesh bottle-1 ((-2 -0.9 0.861667d0) (0 0 0 1))
                                 :mass 0.2 :color (1 0 0) :mesh :bottle))
              (assert (btr:object ?world :mesh bottle-2 ((-0.65 2 0.955) (0 0 0 1))
                                 :mass 0.2 :color (0 1 0) :mesh :bottle))
              (btr:simulate ?world 100))))

(defun move-arm-out-of-sight (&key (arm '(:left :right)))      
  (unless (listp arm)
    (setf arm (list arm)))
  (exe:perform
   (let ((?left-configuration-to-go pr2-pp-plans::*pr2-left-arm-out-of-sight-joint-positions*)
         (?right-configuration-to-go pr2-pp-plans::*pr2-right-arm-out-of-sight-joint-positions*))       
     (desig:a motion
              (type moving-joints)
              (left-configuration ?left-configuration-to-go)
              (right-configuration ?right-configuration-to-go)))))

(defun navigate-to (?navigation-goal)
  (exe:perform (desig:a motion
                        (type going)
                        (target (desig:a location (pose ?navigation-goal))))))

(defun look-at (?point-of-interest)
  (exe:perform (desig:a motion
                              (type looking)
                              (target (desig:a location (pose ?point-of-interest))))))

(defun get-perceived-bottle-desig ()
  (let* ((?bottle-desig (desig:an object
                                      (type bottle)))
             (?perceived-bottle-desig (exe:perform
                                         (desig:a motion
                                                  (type detecting)
                                                  (object ?bottle-desig)))))
    ?perceived-bottle-desig))

(defun pick-up (?object-designator &optional (?arm :right))
  (exe:perform (desig:an action
                                 (type picking-up)
                                 (arm ?arm)
                                 (object ?object-designator))))

(defun place-down (?pose ?arm)
  (exe:perform (desig:an action
                   (type placing)
                   (arm ?arm)
                   (target (desig:a location (pose ?pose))))))

(defun move-to-reach (?object-designator &optional (arm :right) )
  (pr2-pp-plans::drive-towards-object-plan ?object-designator :?arm arm))

(defun test-switch-two-bottles ()
  (unless (assoc :bottle btr::*mesh-files*)
    (add-objects-to-mesh-list))
  (spawn-two-bottles)
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
    (cpl:top-level
      ;; Go to counter top and perceive bottle
      (let ((?navigation-goal pr2-pp-plans::*meal-table-right-base-pose*)
            (?ptu-goal pr2-pp-plans::*meal-table-right-base-look-pose*))
        (cpl:par
          ;; Move torso up
          (exe:perform
           (desig:a motion (type moving-torso) (joint-angle 0.3)))
          (move-arm-out-of-sight)
          (navigate-to ?navigation-goal))
        (look-at ?ptu-goal))
      ;; Pick up bottle-1 with right arm.
      (let* ((?perceived-bottle-desig (get-perceived-bottle-desig)))
        (move-to-reach ?perceived-bottle-desig :right)
        (pick-up ?perceived-bottle-desig :right))
      ;; Move to the meal table
      (let ((?pose *pose-meal-table*))
        (navigate-to ?pose))
      (move-arm-out-of-sight :arm :right)
      ;; Pick up bottle-2 with left arm
      (let* ((?perceived-bottle-desig (get-perceived-bottle-desig)))
        (move-to-reach ?perceived-bottle-desig :left)
        (pick-up ?perceived-bottle-desig :left))
      ;; Move left arm out of sight
      (move-arm-out-of-sight :arm :left)
      ;; Place bottle-1 on second table
      (let ((?drop-pose *pose-bottle-2*))
        (place-down ?drop-pose :right))
      ;; Move right arm out of sight
      (move-arm-out-of-sight :arm :right)
      ;; Move to the counter table 
      (let ((?navigation-goal *pose-counter*))
         (navigate-to ?navigation-goal))
      ;; Place bottle-2 on the counter
      (let ((?drop-pose *pose-bottle-1*))
        (place-down ?drop-pose :left))
      (move-arm-out-of-sight))))
