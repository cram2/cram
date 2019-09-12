;;; Copyright (c) 2019, Christopher Pollok <cpollok@uni-bremen.de>
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(in-package :env-man-tests)

(define-test get-revolute-traj-poses-0-angle
  (let* ((joint-to-gripper
           (cl-transforms-stamped:make-transform-stamped
            "joint"
            "gripper"
            0.0
            (cl-transforms:make-3d-vector 1 0 0)
            (cl-transforms:make-identity-rotation)))
         (axis (cl-transforms:make-3d-vector 0 0 1))
         (traj-poses (env-man::get-revolute-traj-poses
                      joint-to-gripper
                      :axis axis
                      :angle-max 0)))
    (assert-number-equal 0 (length traj-poses))))

(define-test get-revolute-traj-poses-45-angle
  (let* ((joint-to-gripper
           (cl-transforms-stamped:make-transform-stamped
            "joint"
            "gripper"
            0.0
            (cl-transforms:make-3d-vector 1 0 0)
            (cl-transforms:make-identity-rotation)))
         (axis (cl-transforms:make-3d-vector 0 0 1))
         (angle (cram-math:degrees->radians 45))
         (traj-poses (env-man::get-revolute-traj-poses
                      joint-to-gripper
                      :axis axis
                      :angle-max angle))
         (rotated-pos (cl-transforms:normalize-vector
                       (cl-transforms:make-3d-vector 1 1 0)))
         (rotated-rot (cl-transforms:euler->quaternion :az angle)))
    (assert-true (= (ceiling (/ angle 0.1)) (length traj-poses)))
    (assert-true (> 0.1 (acos (cl-transforms:dot-product
                               rotated-pos
                               (cl-transforms:translation
                                (car (last traj-poses)))))))
    (assert-true (> 0.1 (cl-transforms:angle-between-quaternions
                         rotated-rot
                         (cl-transforms:rotation
                          (car (last traj-poses))))))))

(define-test get-revolute-traj-poses-90-angle
  (let* ((joint-to-gripper
           (cl-transforms-stamped:make-transform-stamped
            "joint"
            "gripper"
            0.0
            (cl-transforms:make-3d-vector 1 0 0)
            (cl-transforms:make-identity-rotation)))
         (axis (cl-transforms:make-3d-vector 0 0 1))
         (angle (cram-math:degrees->radians 90))
         (traj-poses (env-man::get-revolute-traj-poses
                      joint-to-gripper
                      :axis axis
                      :angle-max angle))
         (rotated-pos (cl-transforms:make-3d-vector 0 1 0))
         (rotated-rot (cl-transforms:euler->quaternion :az angle)))
    (assert-true (= (ceiling (/ angle 0.1)) (length traj-poses)))
    (assert-true (> 0.1 (acos (cl-transforms:dot-product
                               rotated-pos
                               (cl-transforms:translation
                                (car (last traj-poses)))))))
    (assert-true (> 0.1 (cl-transforms:angle-between-quaternions
                         rotated-rot
                         (cl-transforms:rotation
                          (car (last traj-poses))))))))

(define-test get-revolute-traj-poses-135-angle
  (let* ((joint-to-gripper
           (cl-transforms-stamped:make-transform-stamped
            "joint"
            "gripper"
            0.0
            (cl-transforms:make-3d-vector 1 0 0)
            (cl-transforms:make-identity-rotation)))
         (axis (cl-transforms:make-3d-vector 0 0 1))
         (angle (cram-math:degrees->radians 135))
         (traj-poses (env-man::get-revolute-traj-poses
                      joint-to-gripper
                      :axis axis
                      :angle-max angle))
         (rotated-pos (cl-transforms:normalize-vector
                       (cl-transforms:make-3d-vector -1 1 0)))
         (rotated-rot (cl-transforms:euler->quaternion :az angle))
         )
    (assert-true (= (ceiling (/ angle 0.1)) (length traj-poses)))
    (assert-true (> 0.1 (acos (cl-transforms:dot-product
                               rotated-pos
                               (cl-transforms:translation
                                (car (last traj-poses)))))))
    (assert-true (> 0.1 (cl-transforms:angle-between-quaternions
                         rotated-rot
                         (cl-transforms:rotation
                          (car (last traj-poses))))))
    ))

(define-test get-revolute-traj-poses-180-angle
  (let* ((joint-to-gripper
           (cl-transforms-stamped:make-transform-stamped
            "joint"
            "gripper"
            0.0
            (cl-transforms:make-3d-vector 1 0 0)
            (cl-transforms:make-identity-rotation)))
         (axis (cl-transforms:make-3d-vector 0 0 1))
         (angle (cram-math:degrees->radians 180))
         (traj-poses (env-man::get-revolute-traj-poses
                      joint-to-gripper
                      :axis axis
                      :angle-max angle))
         (rotated-pos (cl-transforms:normalize-vector
                       (cl-transforms:make-3d-vector -1 0 0)))
         (rotated-rot (cl-transforms:euler->quaternion :az angle))
         )
    (assert-true (= (ceiling (/ angle 0.1)) (length traj-poses)))
    (assert-true (> 0.1 (acos (cl-transforms:dot-product
                               rotated-pos
                               (cl-transforms:translation
                                (car (last traj-poses)))))))
    (assert-true (> 0.1 (cl-transforms:angle-between-quaternions
                         rotated-rot
                         (cl-transforms:rotation
                          (car (last traj-poses))))))
    ))


(define-test get-revolute-traj-poses-negative-45-angle
  (let* ((joint-to-gripper
           (cl-transforms-stamped:make-transform-stamped
            "joint"
            "gripper"
            0.0
            (cl-transforms:make-3d-vector 1 0 0)
            (cl-transforms:make-identity-rotation)))
         (axis (cl-transforms:make-3d-vector 0 0 1))
         (angle (cram-math:degrees->radians -45))
         (traj-poses (env-man::get-revolute-traj-poses
                      joint-to-gripper
                      :axis axis
                      :angle-max angle))
         (rotated-pos (cl-transforms:normalize-vector
                       (cl-transforms:make-3d-vector 1 -1 0)))
         (rotated-rot (cl-transforms:euler->quaternion :az angle)))
    (assert-true (= (ceiling (abs (/ angle 0.1))) (length traj-poses)))
    (assert-true (> 0.1 (acos (cl-transforms:dot-product
                               rotated-pos
                               (cl-transforms:translation
                                (car (last traj-poses)))))))
    (assert-true (> 0.1 (cl-transforms:angle-between-quaternions
                         rotated-rot
                         (cl-transforms:rotation
                          (car (last traj-poses))))))))

(define-test get-revolute-traj-poses-negative-90-angle
  (let* ((joint-to-gripper
           (cl-transforms-stamped:make-transform-stamped
            "joint"
            "gripper"
            0.0
            (cl-transforms:make-3d-vector 1 0 0)
            (cl-transforms:make-identity-rotation)))
         (axis (cl-transforms:make-3d-vector 0 0 1))
         (angle (cram-math:degrees->radians -90))
         (traj-poses (env-man::get-revolute-traj-poses
                      joint-to-gripper
                      :axis axis
                      :angle-max angle))
         (rotated-pos (cl-transforms:make-3d-vector 0 -1 0))
         (rotated-rot (cl-transforms:euler->quaternion :az angle)))
    (assert-true (= (ceiling (abs (/ angle 0.1))) (length traj-poses)))
    (assert-true (> 0.1 (acos (cl-transforms:dot-product
                               rotated-pos
                               (cl-transforms:translation
                                (car (last traj-poses)))))))
    (assert-true (> 0.1 (cl-transforms:angle-between-quaternions
                         rotated-rot
                         (cl-transforms:rotation
                          (car (last traj-poses))))))))

(define-test get-revolute-traj-poses-negative-135-angle
  (let* ((joint-to-gripper
           (cl-transforms-stamped:make-transform-stamped
            "joint"
            "gripper"
            0.0
            (cl-transforms:make-3d-vector 1 0 0)
            (cl-transforms:make-identity-rotation)))
         (axis (cl-transforms:make-3d-vector 0 0 1))
         (angle (cram-math:degrees->radians -135))
         (traj-poses (env-man::get-revolute-traj-poses
                      joint-to-gripper
                      :axis axis
                      :angle-max angle))
         (rotated-pos (cl-transforms:normalize-vector
                       (cl-transforms:make-3d-vector -1 -1 0)))
         (rotated-rot (cl-transforms:euler->quaternion :az angle))
         )
    (assert-true (= (ceiling (abs (/ angle 0.1))) (length traj-poses)))
    (assert-true (> 0.1 (acos (cl-transforms:dot-product
                               rotated-pos
                               (cl-transforms:translation
                                (car (last traj-poses)))))))
    (assert-true (> 0.1 (cl-transforms:angle-between-quaternions
                         rotated-rot
                         (cl-transforms:rotation
                          (car (last traj-poses))))))
    ))

(define-test get-revolute-traj-poses-negative-180-angle
  (let* ((joint-to-gripper
           (cl-transforms-stamped:make-transform-stamped
            "joint"
            "gripper"
            0.0
            (cl-transforms:make-3d-vector 1 0 0)
            (cl-transforms:make-identity-rotation)))
         (axis (cl-transforms:make-3d-vector 0 0 1))
         (angle (cram-math:degrees->radians -180))
         (traj-poses (env-man::get-revolute-traj-poses
                      joint-to-gripper
                      :axis axis
                      :angle-max angle))
         (rotated-pos (cl-transforms:normalize-vector
                       (cl-transforms:make-3d-vector -1 0 0)))
         (rotated-rot (cl-transforms:euler->quaternion :az angle))
         )
    (assert-true (= (ceiling (abs (/ angle 0.1))) (length traj-poses)))
    (assert-true (> 0.1 (acos (cl-transforms:dot-product
                               rotated-pos
                               (cl-transforms:translation
                                (car (last traj-poses)))))))
    (assert-true (> 0.1 (cl-transforms:angle-between-quaternions
                         rotated-rot
                         (cl-transforms:rotation
                          (car (last traj-poses))))))
    ))
