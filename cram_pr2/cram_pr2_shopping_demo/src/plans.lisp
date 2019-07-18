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
                       (cl-transforms:make-3d-vector -3.1 0.5 0.75)
                       (cl-transforms:make-identity-rotation)))

(defparameter *pose-grasping* (cl-transforms-stamped:make-pose-stamped
                               "map" 0.0
                               (cl-transforms:make-3d-vector -1.5 -0.3 0)
                               (cl-transforms:axis-angle->quaternion
                                (cl-transforms:make-3d-vector 0 0 1) (/ pi -2))))

(defparameter *pose-grasping-left* (cl-transforms-stamped:make-pose-stamped
                                    "map" 0.0
                                    (cl-transforms:make-3d-vector -1 -0.35 0)
                                    (cl-transforms:axis-angle->quaternion
                                     (cl-transforms:make-3d-vector 0 0 1) (/ pi -2))))

(defparameter *pose-grasping-right* (cl-transforms-stamped:make-pose-stamped
                                     "map" 0.0
                                     (cl-transforms:make-3d-vector -2 -0.35 0)
                                     (cl-transforms:axis-angle->quaternion
                                      (cl-transforms:make-3d-vector 0 0 1) (/ pi -2))))


(defparameter *pose-near-table* (cl-transforms-stamped:make-pose-stamped
                                 "map" 0.0
                                 (cl-transforms:make-3d-vector -2.45 0.5 0)
                                 (cl-transforms:axis-angle->quaternion
                                  (cl-transforms:make-3d-vector 0 0 1) (/ pi 1))))

(defparameter *looking-pose-mid* (cl-transforms-stamped:make-pose-stamped
                                  "map" 0.0
                                  (cl-transforms:make-3d-vector -1.5 -1.05 1.15)
                                  (cl-transforms:make-identity-rotation)))

(defparameter *pose-detecting* (cl-transforms-stamped:make-pose-stamped
                                "map" 0.0
                                (cl-transforms:make-3d-vector -1.5 1 0)
                                (cl-transforms:axis-angle->quaternion
                                 (cl-transforms:make-3d-vector 0 0 1) (/ pi -2))))

(defun move-object (?object ?destination)
  (let ((?grasping-arm :right)
        (?grasp-pose *pose-grasping*)
        (?grasp-pose-left *pose-grasping-left*)
        (?grasp-pose-right *pose-grasping-right*)
        (?pose-near-table *pose-near-table*)
        ?newobject
        ?newtransform)

    (exe:perform
     (desig:a motion
              (type moving-torso)
              (joint-angle 0)))

    (exe:perform
     (desig:an action
               (type going)
               (target (desig:a location
                                (pose ?grasp-pose)))))

    (exe:perform
     (desig:an action
               (type positioning-arm)
               (left-configuration park)
               (right-configuration park)))

    (setf ?newobject (exe:perform
                      (desig:a motion
                               (type world-state-detecting)
                               (object ?object))))

    
    (setf ?newtransform (man-int:get-object-transform ?newobject))
    
    ;; selecting the grasping arm
    (if (< (cl-transforms:y (cl-transforms:translation ?newtransform)) 0)
        (setf ?grasping-arm :right)
        (setf ?grasping-arm :left))

    ;; if the object is to far left move to the left
    (when (> (cl-transforms:y (cl-transforms:translation ?newtransform)) 0.8)
      (setf ?grasping-arm :left)
      (exe:perform (desig:a action
                            (type going)
                            (target (desig:a location
                                             (pose ?grasp-pose-left))))))
    ;; if it is to far on the right move to the right
    (when (< (cl-transforms:y (cl-transforms:translation ?newtransform)) -0.8)
      (setf ?grasping-arm :right)
      (exe:perform (desig:a action
                            (type going)
                            (target (desig:a location
                                             (pose ?grasp-pose-right))))))

    (print ?grasping-arm)

    (exe:perform (desig:an action
                           (type picking-up)
                           (arm ?grasping-arm)
                           (grasp left-side)
                           (object ?newobject)))

    (exe:perform (desig:an action
                           (type going)
                           (target (desig:a location
                                            (pose ?pose-near-table)))))

    ;; If the object was on the top shelf and the torso was lifted
    (exe:perform (desig:a motion
                          (type moving-torso)
                          (joint-angle 0)))

    (exe:perform (desig:an action
                           (type placing)
                           (arm ?grasping-arm)
                           (object ?object)
                           (target (desig:a location
                                            (pose ?destination)))))))
(defun collect-article ()
  (urdf-proj:with-simulated-robot
    (let ((objects '(:heitmann :dove :denkmit))
          (y 0.2)
          object-desigs
          destination)
      (setf object-desigs (try-detecting objects))
      (loop for ?object in object-desigs
            do (setf destination (cl-transforms-stamped:make-pose-stamped
                                  "map" 0
                                  (cl-transforms:make-3d-vector -3.1 y 0.75)
                                  (cl-transforms:make-identity-rotation)))
               (move-object ?object destination)
               (btr:simulate btr:*current-bullet-world* 100)
               (setf y (+ y 0.15))))))

(defun try-detecting (articles)
  (let ((?pose-detecting *pose-detecting*)
        (?percived-objects '())
        (?pose-grasping *pose-grasping*))

    (exe:perform (desig:an action
                           (type going)
                           (target (desig:a location
                                            (pose ?pose-detecting)))))

    (exe:perform (desig:a motion
                          (type moving-torso)
                          (joint-angle 0)))

    ;; Tries to detect every object in articles
    (loop for ?article in articles
          do (push
              (exe:perform (desig:a motion
                                    (type detecting)
                                    (object (desig:an object
                                                      (type ?article)))))
              ?percived-objects))

    (exe:perform (desig:an action
                           (type going)
                           (target (desig:a location
                                            (pose ?pose-grasping)))))

    ?percived-objects))
