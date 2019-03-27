;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(defparameter *robot-x-position* -0.15)
(defparameter *robot-y-position* 1.0)
(defparameter *object-x-position* -0.75)
(defparameter *object-y-position* 1.0)
(defparameter *object-z-position* 0.8573)

(defun generate-training-data (&optional debug-mode
                                 (object-list '(:bowl :spoon :cup :milk :breakfast-cereal)))
  (pr2-proj:with-simulated-robot

    (exe:perform
     (desig:an action
               (type positioning-arm)
               (left-configuration park)
               (right-configuration park)))

    (let ((?pose (cl-transforms-stamped:make-pose-stamped
                  "map" 0.0
                  (cl-transforms:make-3d-vector *robot-x-position* *robot-y-position* 0)
                  (cl-transforms:make-quaternion 0 0 1 0))))
      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location (pose ?pose))))))

    (dotimes (i 4)
      (pr2-proj::look-at-pose-stamped
       (cl-transforms-stamped:make-pose-stamped
                  "base_footprint" 0.0
                  (cl-transforms:make-3d-vector 0.5 0 0.7)
                  (cl-transforms:make-identity-rotation))))
    (pr2-proj::move-torso 0.15)

    (btr:detach-all-objects (btr:get-robot-object))
    (btr-utils:kill-all-objects)

    (setf cram-pr2-projection::*ik-solution-cache*
          (make-hash-table :test 'cram-pr2-projection::arm-poses-equal-accurate))

    (when debug-mode
      (btr-utils:spawn-object 'red-dot
                              :pancake-maker
                              :color '(1 0 0 0.5)
                              :pose '((0.0 0.0 -1.0) (0 0 0 1)))
      (btr-utils:spawn-object 'green-dot
                              :pancake-maker
                              :color '(0 1 0 0.5)
                              :pose '((0.0 0.0 -1.0) (0 0 0 1)))
      (setf pr2-proj::*debug-long-sleep-duration* 0.5)
      (setf pr2-proj::*debug-short-sleep-duration* 0.1))

    (unwind-protect
         ;; for every object in the object-list
         (dolist (?object-type object-list)
           (format t "OBJECT: ~a~%~%" ?object-type)
           ;; call the garbage collector to clean up memory
           (sb-ext:gc :full t)
           ;; spawn the object at world's origin
           (let ((btr-object (btr:add-object btr:*current-bullet-world*
                                             :mesh
                                             'object-to-grasp
                                             (cl-transforms:make-identity-pose)
                                             :mesh ?object-type
                                             :mass 0.2
                                             :color '(1 0 0))))

             ;; try 12 different orientations
             (dolist (rotation-axis (list (cl-transforms:make-3d-vector 1 0 0)
                                          (cl-transforms:make-3d-vector 0 1 0)
                                          (cl-transforms:make-3d-vector 0 0 1)))
               (dolist (rotation-angle (list (* pi 0.0)
                                             (* pi 0.5)
                                             (* pi 1.0)
                                             (* pi 1.5)))
                 (format t "ORIENTATION: ~a ~a~%~%" rotation-axis rotation-angle)
                 (let* ((orientation (cl-transforms:axis-angle->quaternion
                                      rotation-axis rotation-angle))
                        (pose-for-bb-calculation (cl-transforms:make-pose
                                                  (cl-transforms:make-3d-vector 0 0 -1)
                                                  orientation)))
                   (setf (btr:pose btr-object) pose-for-bb-calculation)
                   (let* ((bb-dims (cl-bullet:bounding-box-dimensions
                                    (cl-bullet:aabb btr-object)))
                          (z/2 (/ (cl-transforms:z bb-dims) 2))
                          (position (cl-transforms:make-3d-vector
                                     *object-x-position*
                                     *object-y-position*
                                     (+ *object-z-position* z/2)))
                          (pose-for-grasping (cl-transforms:make-pose
                                              position
                                              orientation)))
                     (setf (btr:pose btr-object) pose-for-grasping)
                     (when debug-mode (cpl:sleep 0.5))

                     ;; check if orientation is stable
                     (btr:simulate btr:*current-bullet-world* 10)
                     (if (or (> (abs (cl-transforms:normalize-angle
                                      (cl-transforms:angle-between-quaternions
                                       (cl-transforms:orientation
                                        (btr:pose btr-object))
                                       orientation)))
                                1.0)
                             (> (cl-transforms:v-dist
                                 (cl-transforms:origin
                                  (btr:pose btr-object))
                                 position)
                                0.1))
                         (when debug-mode
                           (format t "~a with orientation ~a unstable.~%Skipping...~%"
                                   ?object-type orientation)
                           (btr-utils:move-object 'red-dot '((-1.0 2.0 1.0) (0 0 0 1)))
                           (btr-utils:move-object 'green-dot '((0.0 0.0 -1.0) (0 0 0 1)))
                           (cpl:sleep 0.5))

                         ;; try 9 different base positions
                         (dolist (position-x-offset '(0 -0.15 0.15))
                           (dolist (position-y-offset '(0 -0.25 0.25))
                             ;; with each of the arms
                             (dolist (?arm '(:left :right))
                               ;; and each of available grasps
                               (dolist (?grasp (man-int:get-object-type-grasps
                                                ?object-type ?arm nil))

                                 ;; detach object, move object, move robot, park arms
                                 (btr:detach-all-objects (btr:get-robot-object))
                                 (setf (btr:pose btr-object)
                                       (cl-transforms:make-pose
                                        (cl-transforms:make-3d-vector
                                         *object-x-position*
                                         *object-y-position*
                                         (+ *object-z-position* z/2))
                                        orientation))
                                 (let ((?robot-pose
                                         (cl-transforms-stamped:make-pose-stamped
                                          "map" 0.0
                                          (cl-transforms:make-3d-vector
                                           (+ *robot-x-position* position-x-offset)
                                           (+ *robot-y-position* position-y-offset)
                                           0)
                                          (cl-transforms:axis-angle->quaternion
                                           (cl-transforms:make-3d-vector 0 0 1)
                                           (costmap::angle-to-point-direction
                                            (+ *robot-x-position* position-x-offset)
                                            (+ *robot-y-position* position-y-offset)
                                            (cl-transforms:make-3d-vector
                                             -0.75 1.0 (+ 0.8573 z/2)))))))
                                   (exe:perform
                                    (desig:an action
                                              (type positioning-arm)
                                              (left-configuration park)
                                              (right-configuration park)))
                                   (exe:perform
                                    (desig:a motion
                                             (type moving-torso)
                                             (joint-angle 0.15)))
                                   (exe:perform
                                    (desig:an action
                                              (type going)
                                              (target (desig:a location (pose ?robot-pose))))))

                                 (when debug-mode
                                   (btr-utils:move-object 'green-dot '((-1.0 2.0 1.0) (0 0 0 1)))
                                   (btr-utils:move-object 'red-dot '((0.0 0.0 -1.0) (0 0 0 1)))
                                   (cpl:sleep 0.5))

                                 (cpl:with-failure-handling
                                     ((cram-language:simple-plan-failure (e)
                                        (when debug-mode
                                          (format t "Error happened: ~a~%Ignoring..." e)
                                          (btr-utils:move-object 'red-dot
                                                                 '((-1.0 2.0 1.0) (0 0 0 1)))
                                          (btr-utils:move-object 'green-dot
                                                                 '((0.0 0.0 -1.0) (0 0 0 1)))
                                          (cpl:sleep 0.5))
                                        (return)))
                                   (let* ((?object-designator
                                            (exe:perform
                                             (desig:an action
                                                       (type detecting)
                                                       (object (desig:an object
                                                                         (type ?object-type)))))))
                                     (exe:perform (desig:an action
                                                            (type picking-up)
                                                            (arm ?arm)
                                                            (grasp ?grasp)
                                                            (object ?object-designator)))
                                     (when debug-mode (cpl:sleep 0.5)))))))))))))
             (btr:remove-object btr:*current-bullet-world* 'object-to-grasp)))
      (when debug-mode
        (btr-utils:kill-object 'red-dot)
        (btr-utils:kill-object 'green-dot)
        (setf pr2-proj::*debug-long-sleep-duration* 0.5)
        (setf pr2-proj::*debug-short-sleep-duration* 0.1)))))
