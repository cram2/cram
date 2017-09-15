;; Rotations in 2D and 3D
;; Liam Healy Mon Sep  4 2000 - 17:05
;; Time-stamp: <2011-05-23 22:50:55EDT rotation.lisp>

;; Copyright 2011 Liam M. Healy
;; Distributed under the terms of the GNU General Public License
;;
;; This program is free software: you can redistribute it and/or modify
;; it under the terms of the GNU General Public License as published by
;; the Free Software Foundation, either version 3 of the License, or
;; (at your option) any later version.
;;
;; This program is distributed in the hope that it will be useful,
;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;; GNU General Public License for more details.
;;
;; You should have received a copy of the GNU General Public License
;; along with this program.  If not, see <http://www.gnu.org/licenses/>.

(in-package :antik)

(export '(rotate rotate-3d rotate-3daa
	  euler-angle-rotation tait-bryan-rotation tait-bryan-angles))

;; These are Givens rotations in 2 and 3 space.

(defun rotate (angle &optional passive)
  "Create a rotation matrix in two dimensions.
   Passive rotation if passive is true, otherwise active."
  (with-pq ((angle radian))
    (let* ((mat (grid:make-simple-grid :dimensions '(2 2) :initial-element 0.0d0)))
      (setf (grid:aref mat 0 0) (cos angle)
	    (grid:aref mat 0 1) (* (if passive 1 -1) (sin angle))
	    (grid:aref mat 1 0) (* (if passive -1 1) (sin angle))
	    (grid:aref mat 1 1) (cos angle))
      mat)))

(defun rotate-3d (axis angle &optional passive)
  "Create a rotation matrix about the (0, 1, 2)
   axis in three dimensions.  Passive rotation
   if passive is true, otherwise active."
  (with-pq ((angle radian))
    (let ((2d (rotate
	       angle
	       ;; y-axis rotation is the opposite sense
	       (if (member axis '(1 :y)) (not passive) passive)))
	  (3d (grid:identity-matrix 3))
	  (axis0
	   (case axis ((0 :x) 1) ((1 :y) 0) ((2 :z) 0)))
	  (axis1
	   (case axis ((0 :x) 2) ((1 :y) 2) ((2 :z) 1))))
      (setf (grid:aref 3d axis0 axis0) (grid:aref 2d 0 0)
	    (grid:aref 3d axis0 axis1) (grid:aref 2d 0 1)
	    (grid:aref 3d axis1 axis0) (grid:aref 2d 1 0)
	    (grid:aref 3d axis1 axis1) (grid:aref 2d 1 1))
      3d)))

(defun euler-angle-rotation (rot3a rot1 rot3b &optional passive)
  "Compute the matrix for a 3-1-3 rotation.  See Curtis (9.118)."
  (with-pq ((rot3a radian) (rot1 radian) (rot3b radian))
    (*
     (rotate-3d :z rot3a passive)
     (* (rotate-3d :x rot1 passive)
	(rotate-3d :z rot3b passive)))))

;;; See notes [[id:675d6801-c5fe-4857-9846-6c29f81ab769][Factorizing 321]] 
;;; See 2006 Diebel http://citeseer.ist.psu.edu/viewdoc/summary?doi%3D10.1.1.110.5134
;;; or 2010 Curtis http://www.worldcat.org/oclc/609997774  p. 529, and computed with Maxima.
;;; φ as long as cθ≠0
;;; θ, half circle answer only
;;; ψ as long as cθ≠0

(defun tait-bryan-angles (DCM)
  "Find the three angles psi, theta, phi of a 3-2-1 (Tait-Bryan)
   rotation for the direction cosine matrix."
  ;; See 2006 Diebel (67) or Maxima output
  (values (atan (grid:aref DCM 1 2) (grid:aref DCM 2 2))
	  (atan (- (grid:aref DCM 0 2))
		(sqrt (pythagorean-sum (grid:aref DCM 0 0) (grid:aref DCM 0 1))))
	  (atan (grid:aref DCM 0 1) (grid:aref DCM 0 0))))

(defun tait-bryan-rotation (rot1 rot2 rot3 &optional passive)
  "Compute the matrix for a 3-2-1 (yaw-pitch-roll) rotation.
   See Curtis (9.124).  This transforms a vector in the body
   frame into a vector in the external frame."
  (with-pq ((rot3 radian) (rot2 radian) (rot1 radian))
    (*
     (rotate-3d :x rot1 passive)
     (* (rotate-3d :y rot2 passive)
	(rotate-3d :z rot3 passive)))))

(defun rotate-3daa (rotation-axis angle &optional passive)
  "Rotation about an arbitrary axis in 3D.
   See Goldstein, Poole, Safko (4.62) or
   http://mathworld.wolfram.com/RotationFormula.html."
  ;; Their formula is active, but rotation is defined clockwise.
  (with-pq ((angle radian))
    (let ((ang (if passive angle (- angle))))
      (+ (grid:identity-matrix 3 (cos ang))
	 (* (- 1.0 (cos ang))
	    ;; Dot product.
	    (grid:matrix-from-columns
	     (* rotation-axis (grid:aref rotation-axis 0))
	     (* rotation-axis (grid:aref rotation-axis 1))
	     (* rotation-axis (grid:aref rotation-axis 2))))
	 (* (sin ang)
	    (let ((xpmat
		   (grid:make-simple-grid
		    :dimensions '(3 3) :initial-element 0.0d0)))
	      ;; cross product
	      (setf (grid:aref xpmat 0 1) (grid:aref rotation-axis 2)
		    (grid:aref xpmat 0 2) (- (grid:aref rotation-axis 1))
		    (grid:aref xpmat 1 0) (- (grid:aref rotation-axis 2))
		    (grid:aref xpmat 1 2) (grid:aref rotation-axis 0)
		    (grid:aref xpmat 2 0) (grid:aref rotation-axis 1)
		    (grid:aref xpmat 2 1) (- (grid:aref rotation-axis 0)))
	      xpmat))))))

;;; Need to add pythagorean-complement
#+(or)
(defun euler-parameters (rotation-matrix)
  "The unique rotation angle and axis for this matrix."
  ;; Originally got this from Hall's class web page?
  ;; But had to change sign on the rotation vector
  ;; so that it represented an active rotation.
  (let ((arg
	 (/ (+ (grid:aref rotation-matrix 0 0)
	       (grid:aref rotation-matrix 1 1)
	       (grid:aref rotation-matrix 2 2)
	       -1)
	    2)))
    (values
      (acos arg)
      (/ (vector
	  (- (grid:aref rotation-matrix 2 1) (grid:aref rotation-matrix 1 2))
	  (- (grid:aref rotation-matrix 0 2) (grid:aref rotation-matrix 2 0))
	  (- (grid:aref rotation-matrix 1 0) (grid:aref rotation-matrix 0 1)))
	 (* 2 (pythagorean-complement arg))))))

;;; This should be defined as a generalized number perhaps.
(defun quaternion-euler (angle axis)
  "The quaternion corresponding to a given pair of Euler parameters."
  (vector
   (cos (/ angle 2))
   (* (sin (/ angle 2)) (grid:aref axis 0))
   (* (sin (/ angle 2)) (grid:aref axis 1))
   (* (sin (/ angle 2)) (grid:aref axis 2))))

(defun euler-quaternion (quaternion)
  "The Euler angle and axis for the quaternion."
  (let* ((vector (grid:subgrid quaternion '(3) '(1)))
	 (angle (* 2 (atan (grid:norm vector) (grid:aref quaternion 0)))))
    (values
     angle
     (/ vector (sin (/ angle 2))))))	; watch out for zero division

#+(or)
(defun quaternion (rotation-matrix)
  "The quaternion corresponding to this rotation matrix."
  (multiple-value-bind (ang axis)
      (euler-parameters rotation-matrix)
    (quaternion-euler ang axis)))

(defun quaternion* (q1 q2)
  "Quaternion multiplication."
  (grid:make-simple-grid
   :initial-contents
   (list
    (+ (* (elt q1 0) (elt q2 0))
       (* -1 (elt q1 1) (elt q2 1))
       (* -1 (elt q1 2) (elt q2 2))
       (* -1 (elt q1 3) (elt q2 3)))
    (+ (* (elt q1 1) (elt q2 0))
       (* (elt q1 2) (elt q2 3))
       (* -1 (elt q1 3) (elt q2 2))
       (* (elt q1 0) (elt q2 1)))
    (+ (* -1 (elt q1 1) (elt q2 3))
       (* (elt q1 2) (elt q2 0))
       (* (elt q1 3) (elt q2 1))
       (* (elt q1 0) (elt q2 2)))
    (+ (* (elt q1 1) (elt q2 2))
       (* -1 (elt q1 2) (elt q2 1))
       (* (elt q1 3) (elt q2 0))
       (* (elt q1 0) (elt q2 3))))))

;;; (setf R1 (euler-angle-rotation #_24_deg 0 0))
;;; #2A((0.9135454576426009 -0.4067366430758002 0.0) (0.4067366430758002 0.9135454576426009 0.0) (0.0 0.0 1.0))
;;; (setf R2 (euler-angle-rotation 0 #_34_deg 0))
;;; (quaternion (* R1 R2))
;;; (quaternion* (quaternion R1) (quaternion R2))

;;; Better reference (not only unit quaternions)
;;; http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
(defun direction-cosine (q)
  "Compute the rotation (direction cosine) matrix (DCM) from the unit quaternion."
  ;; From van den Bergen, "Collision Detection in Interactive 3D environments", p. 45. 
  (let ((ans (grid:make-simple-grid :dimensions '(3 3))))
    (setf
     (grid:aref ans 0 0)
     (- 1 (* 2 (+ (expt (grid:aref q 2) 2) (expt (grid:aref q 3) 2))))
     (grid:aref ans 1 1)
     (- 1 (* 2 (+ (expt (grid:aref q 1) 2) (expt (grid:aref q 3) 2))))
     (grid:aref ans 2 2)
     (- 1 (* 2 (+ (expt (grid:aref q 1) 2) (expt (grid:aref q 2) 2))))
     (grid:aref ans 0 1)
     (* 2 (- (* (grid:aref q 1) (grid:aref q 2)) (* (grid:aref q 0) (grid:aref q 3))))
     (grid:aref ans 1 0)
     (* 2 (+ (* (grid:aref q 1) (grid:aref q 2)) (* (grid:aref q 0) (grid:aref q 3))))
     (grid:aref ans 0 2)
     (* 2 (+ (* (grid:aref q 1) (grid:aref q 3)) (* (grid:aref q 0) (grid:aref q 2))))
     (grid:aref ans 2 0)
     (* 2 (- (* (grid:aref q 1) (grid:aref q 3)) (* (grid:aref q 0) (grid:aref q 2))))
     (grid:aref ans 2 1)
     (* 2 (+ (* (grid:aref q 2) (grid:aref q 3)) (* (grid:aref q 0) (grid:aref q 1))))
     (grid:aref ans 1 2)
     (* 2 (- (* (grid:aref q 2) (grid:aref q 3)) (* (grid:aref q 0) (grid:aref q 1)))))
    ans))

;;; (setf q1 (quaternion r1))
;;; (0.9781476007338056 0.0 0.0 0.20791169081775937)
;;; (direction-cosine q1)
;;; #2A((0.9135454576426009 -0.40673664307580026 0.0) (0.40673664307580026 0.9135454576426009 0.0) (0.0 0.0 1.0))
