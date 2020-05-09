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
   (btr:get-environment-object)
   'cram-bullet-reasoning:urdf))

(defun move-kitchen-joint (&key (joint-name "iai_fridge_door_joint")
                             (joint-angle 0.2d0))
  (btr:set-robot-state-from-joints
   `((,joint-name  ,joint-angle))
   (btr:get-environment-object)))

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

(defparameter *pose-bottle-1*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 -0.9d0 0.86d0)
   (cl-transforms:make-identity-rotation)))

(defparameter *pose-bottle-2*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -0.8 2 0.9)
   (cl-transforms:make-identity-rotation)))

(defparameter *pose-meal-table*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -0.15 2.0 0)
   (cl-transforms:make-quaternion 0.0d0 0.0d0 -1.0d0 0.0d0)))

(defparameter *pose-counter*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -1.8547d0 -0.381d0 0.0d0)
   (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi -2))))

(defun spawn-two-bottles ()
  (unless (assoc :bottle btr::*mesh-files*)
    (add-objects-to-mesh-list))
  (prolog:prolog
   `(and (btr:bullet-world ?world)
         (assert (btr:object ?world :mesh bottle-1 ((-2 -0.9 0.860) (0 0 0 1))
                             :mass 0.2 :color (1 0 0) :mesh :bottle))
         (assert (btr:object ?world :mesh bottle-2 ((-0.8 2 0.9) (0 0 0 1))
                             :mass 0.2 :color (0 1 0) :mesh :bottle))
         (btr:simulate ?world 100))))

(defun navigate-to (?navigation-goal)
  (exe:perform (desig:a motion
                        (type going)
                        (pose ?navigation-goal))))

(defun look-at (?point-of-interest)
  (exe:perform (desig:a motion
                        (type looking)
                        (pose ?point-of-interest))))

(defun get-perceived-bottle-desig ()
  (let* ((?bottle-desig (desig:an object (type bottle)))
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

(defun place-down (?pose ?object ?arm)
  (exe:perform (desig:an action
                         (type placing)
                         (arm ?arm)
                         (object ?object)
                         (target (desig:a location (pose ?pose))))))

(defun test-switch-two-bottles ()
  (spawn-two-bottles)
  (proj:with-projection-environment urdf-proj:urdf-bullet-projection-environment
    (cpl:top-level
      ;; Go to counter top and perceive bottle
      (let ((?navigation-goal *pose-counter*)
            (?ptu-goal 
              (cl-transforms-stamped:make-pose-stamped
               "base_footprint"
               0.0
               (cl-transforms:make-3d-vector 0.65335d0 0.076d0 0.758d0)
               (cl-transforms:make-identity-rotation))))
        (cpl:par
          ;; Move torso up
          (exe:perform
           (desig:a motion (type moving-torso) (joint-angle 0.3)))
          (exe:perform
           (desig:an action
                     (type parking-arms)))
          (navigate-to ?navigation-goal))
        (look-at ?ptu-goal))
      ;; Pick up bottle-1 with right arm.
      (let ((?perceived-bottle-1 (get-perceived-bottle-desig)))
        (pick-up ?perceived-bottle-1 :right)
        (exe:perform
         (desig:an action
                   (type parking-arms)
                   (arms (:right))))
        ;; Move to the meal table
        (let ((?pose *pose-meal-table*))
          (navigate-to ?pose))
        ;; Pick up bottle-2 with left arm
        (let ((?perceived-bottle-2 (get-perceived-bottle-desig)))
          (pick-up ?perceived-bottle-2 :left)
          ;; Move left arm out of sight
          (exe:perform
           (desig:an action
                     (type parking-arms)
                     (arms (:left))))
          ;; Place bottle-1 on second table
          (let ((?drop-pose *pose-bottle-2*))
            (place-down ?drop-pose ?perceived-bottle-1 :right))
          ;; Move right arm out of sight
          (exe:perform
           (desig:an action
                     (type parking-arms)
                     (arms (:right))))
          ;; Move to the counter table
          (let ((?navigation-goal *pose-counter*))
            (navigate-to ?navigation-goal))
          ;; Place bottle-2 on the counter
          (let ((?drop-pose *pose-bottle-1*))
            (place-down ?drop-pose ?perceived-bottle-2 :left))
          (exe:perform
           (desig:an action
                     (type parking-arms))))))))
