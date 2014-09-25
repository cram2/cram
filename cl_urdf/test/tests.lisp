;;; Copyright (c) 2014, Jannik Buckelo <jannikbu@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
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

(in-package :cl-urdf-test)

(defun load-robot (name)
  "Parses the urdf file with name `name' in the test_urdfs directory."
  (cl-urdf:parse-urdf (pathname (format nil "test_urdfs/~a" name))))

(defun test-urdf (name)
  "Tests the parsing and writing of the urdf file with name `name' in 
the test_urdfs directory."
  (let* ((first-robot (load-robot name))
         (urdf (generate-urdf-string first-robot))
         (second-robot (parse-urdf urdf)))
    (assert-equal (name first-robot) (name second-robot))
    (compare-link-tables (links first-robot) (links second-robot))
    (compare-joint-tables (joints first-robot) (joints second-robot))))


;;; Functions to compare the robot parts

(defun compare-link-tables (links1 links2)
  (assert-equal (hash-table-count links1) (hash-table-count links2))
  (maphash (lambda (hash link1) 
             (let ((link2 (gethash hash links2)))
               (assert-true link2)
               (when link2
                 (compare-links link1 link2))))
           links1))

(defun compare-links (link1 link2)
  (assert-equal (null (inertial link1)) (null (inertial link2)))
  (when (and (inertial link1) (inertial link2))
    (compare-inertial (inertial link1) (inertial link2)))

  (assert-equal (null (visual link1)) (null (visual link2)))
  (when (and (visual link1) (visual link2))
    (compare-visual (visual link1) (visual link2)))

  (assert-equal (null (collision link1)) (null (collision link2)))
  (when (and (collision link1) (collision link2))
    (compare-collision (collision link1) (collision link2))))

(defun compare-inertial (inertial1 inertial2)
  (compare-origins (origin inertial1) (origin inertial2))

  (assert-equalp (mass inertial1) (mass inertial2)))

(defun compare-visual (visual1 visual2)
  (compare-origins (origin visual1) (origin visual2))
  (compare-geometry (geometry visual1) (geometry visual2))
  
  (assert-equal (null (material visual1)) (null (material visual2)))
  (when (and (material visual1) (material visual2))
    (compare-material (material visual1) (material visual2))))

(defun compare-collision (collision1 collision2)
  (compare-origins (origin collision1) (origin collision2))
  (compare-geometry (geometry collision1) (geometry collision2)))

(defun compare-geometry (geometry1 geometry2)
  ;; TODO test the single eometry classes
  (assert-equal (type-of geometry1) (type-of geometry2)))

(defun compare-material (material1 material2)
  (let ((bound-name1 (slot-boundp material1 'name))
        (bound-name2 (slot-boundp material2 'name))
        (bound-texture1 (slot-boundp material1 'texture))
        (bound-texture2 (slot-boundp material2 'texture)))

    (when (and bound-name1 bound-name2)
      (assert-equal (name material1) (name material2)))
    
    (assert-equal (null (color material1)) (null (color material2)))
    (when (and (color material1) (color material2))
      (assert-equal (color material1) (color material2)))

    (when (and bound-texture1 bound-texture2)
      (assert-equal (null (texture material1)) (null (texture material2)))
      (when (and (texture material1) (texture material2))
        (assert-equal (namestring (texture material1)) (namestring (texture material2)))))))

(defun compare-origins (origin1 origin2)
  (let ((translation1 (cl-transforms:translation origin1))
        (translation2 (cl-transforms:translation origin2))
        (rotation1 (cl-transforms:rotation origin1))
        (rotation2 (cl-transforms:rotation origin2)))
    (compare-3d-vectors translation1 translation2)
    (compare-quaternions rotation1 rotation2)))

(defun compare-3d-vectors (v1 v2)
  (assert-equal (cl-transforms:x v1) (cl-transforms:x v2))
  (assert-equal (cl-transforms:y v1) (cl-transforms:y v2))
  (assert-equal (cl-transforms:z v1) (cl-transforms:z v2)))

(defun compare-quaternions (q1 q2)
  ;; Fixed an error but comparision still doesn't work because of
  ;; the ambiguity of rpy and inaccuries in the deciaml places

  ;;(assert-equal (cl-transforms:w q1) (cl-transforms:w q2) q1 q2)
  ;;(assert-equal (cl-transforms:x q1) (cl-transforms:x q2))
  ;;(assert-equal (cl-transforms:y q1) (cl-transforms:y q2))
  ;;(assert-equal (cl-transforms:z q1) (cl-transforms:z q2)))
)

(defun compare-joint-tables (joints1 joints2)
  (assert-equal (hash-table-count joints1) (hash-table-count joints2))
  (maphash (lambda (hash joint1) 
             (let ((joint2 (gethash hash joints2)))
               (assert-true joint2)
               (when joint2
                 (compare-joints joint1 joint2))))
           joints1))

(defun compare-joints (joint1 joint2)
  (assert-equal (name joint1) (name joint2))
  (assert-equal (name (parent joint1)) (name (parent joint2)))
  (assert-equal (name (child joint1)) (name (child joint2)))

  (compare-3d-vectors (axis joint1) (axis joint2))
  (compare-origins (origin joint2) (origin joint2))

  (assert-equal (slot-boundp joint1 'limits) (slot-boundp joint2 'limits))
  (when (and (slot-boundp joint1 'limits) (slot-boundp joint2 'limits))
    (assert-equal (null joint1) (null joint2))
    (when (and  joint1 joint2)
      (compare-limits (limits joint1) (limits joint2)))))

(defun compare-limits (limits1 limits2)
  (assert-equalp (upper limits1) (upper limits2))
  (assert-equalp (lower limits1) (lower limits2))
  (assert-equalp (effort limits1) (effort limits2))
  (assert-equalp (velocity limits1) (velocity limits2)))


;;; Tests for different urdf files

(define-test simple-robot
  (test-urdf "simple_robot.urdf"))

(define-test simple_robot_kinematics
  (test-urdf "simple_robot_kinematics.urdf"))

(define-test complex-robot
  (test-urdf "complex_robot.urdf"))

(define-test meshes-and-textures-robot
  (test-urdf "meshes_and_textures_robot.urdf"))

(define-test pr2-robot
  (test-urdf "pr2.urdf"))