;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;

(in-package :btr)

(defun constant-sequence (value)
  (lazy-list () (cont value)))

(defun random-number-sequence (lower-bound upper-bound)
  "Returns an infinite lazy list of random numbers between
`lower-bound' and `upper-bound'."
  (assert (>= upper-bound lower-bound))
  (if (= upper-bound lower-bound)
      (constant-sequence lower-bound)
      (lazy-list ()
        (cont (+ lower-bound (random (- upper-bound lower-bound)))))))

(defun number-sequence (lower-bound upper-bound)
  "Returns the sequence of integers from `lower-bound' to
  `upper-bound', excluding `upper-bound'"
  (lazy-list ((i lower-bound))
    (when (< i upper-bound)
      (cont i (+ i 1)))))

(defun random-points-in-sphere (radius)
  (lazy-mapcar
   (lambda (radius angle angle-2)
     (cl-transforms:make-3d-vector
      (* radius (sin angle))
      (* radius (cos angle))
      (* radius (sin angle-2))))
   (random-number-sequence 0 (float radius))
   (random-number-sequence 0 (* pi 2))
   (random-number-sequence 0 (* pi 2))))

(defun random-points-on-circle (radius)
  "Randomly generates random points on a circle with `radius' that lies on
the x-y plane."
  (lazy-mapcar
   (lambda (radius angle)
     (cl-transforms:make-3d-vector
      (* radius (sin angle))
      (* radius (cos angle))
      0.0))
   (random-number-sequence 0 (float radius))
   (random-number-sequence 0 (* pi 2))))

(defun random-points-in-box (dimensions)
  "Randomly generates points in a box of the size `dimensions'"
  (let ((dimensions (ensure-vector dimensions)))
    (lazy-mapcar
     #'cl-transforms:make-3d-vector
     (random-number-sequence (/ (cl-transforms:x dimensions) -2)
                             (/ (cl-transforms:x dimensions) 2))
     (random-number-sequence (/ (cl-transforms:y dimensions) -2)
                             (/ (cl-transforms:y dimensions) 2))
     (random-number-sequence (/ (cl-transforms:z dimensions) -2)
                             (/ (cl-transforms:z dimensions) 2)))))

(defun points-in-box (dimensions n-x n-y n-z)
  "Generates the sequence of points inside the box described by
  `dimensions'. The number of points along the x-axis is `n-x', along
  the y-axis `n-y' and along the z-axis `n-z'."
  (let* ((dimensions (ensure-vector dimensions))
         (step-x (/ (cl-transforms:x dimensions) n-x))
         (step-y (/ (cl-transforms:y dimensions) n-y))
         (step-z (/ (cl-transforms:z dimensions) n-z)))
    (lazy-mapcar
     (curry #'apply #'cl-transforms:make-3d-vector)
     (lazy-cross-product
      (lazy-mapcar
       (curry #'* step-x)
       (number-sequence (/ n-x -2)
                        (+ 1.0 (/ n-x 2))))
      (lazy-mapcar
       (curry #'* step-y)
       (number-sequence (/ n-y  -2)
                        (+ 1.0 (/ n-y 2))))
      (lazy-mapcar
       (curry #'* step-z)
       (number-sequence (/ n-z -2)
                        (+ 1.0 (/ n-z 2))))))))

(defun random-points-along-line (start-point step &optional max-distance)
  "Generates a sequence of points along the line starting at
`start-point'"
  (let ((start-point (ensure-vector start-point))
        (step (ensure-vector step)))
    (assert (> (cl-transforms:v-norm step) 0))
    (if max-distance
        (let ((max-steps (truncate (/ max-distance (cl-transforms:v-norm step)))))
          (lazy-list ((curr start-point)
                      (n max-steps))
            (when (> n 0)
              (cont curr (cl-transforms:v+ curr step) (- n 1)))))
        (lazy-list ((curr start-point))
          (cont curr (cl-transforms:v+ curr step))))))

(defun random-points-on-line (start-point direction max-distance)
  "Randomly samples points on the line given by the vectors
`startp-point' and `direction'. Only points between (START-POINT -
MAX-DISTANCE * DIRECTION) and (START-POINT + MAX-DISTANCE * DIRECTION)
are generated"
  (let ((start-point (ensure-vector start-point))
        (direction (ensure-vector direction)))
    (let ((direction-normalized (cl-transforms:v*
                                 direction
                                 (/ (cl-transforms:v-norm direction)))))
      (lazy-mapcar (lambda (n)
                     (cl-transforms:v+
                      start-point
                      (cl-transforms:v* direction-normalized n)))
                   (random-number-sequence (- max-distance) max-distance)))))

(defun random-poses-on (bottom top)
  (poses-on bottom top #'random-points-in-box))

(defun n-poses-on (bottom top n)
  (let ((n-x (sqrt n))
        (n-y (sqrt n)))
    (poses-on bottom top (rcurry #'points-in-box n-x n-y 1))))

(defun poses-on (bottom top &optional (generator #'random-points-in-box))
  "Returns a sequence of points on the object `bottom'. This function
only takes into account the bounding box and fixes the z-value such
that the bounding boxes of `bottom' and `top' are alligned."
  (let* ((aabb-bottom (aabb bottom))
         (aabb-top (aabb top))
         (ref (cl-transforms:v+
               (bounding-box-center aabb-bottom)
               (cl-transforms:make-3d-vector
                0 0 (/ (cl-transforms:z (bounding-box-dimensions aabb-bottom)) 2))
               (cl-transforms:make-3d-vector
                0 0 (/ (cl-transforms:z (bounding-box-dimensions aabb-top)) 2)))))
    (lazy-mapcar
     (compose (lambda (pt)
                (cl-transforms:make-pose
                 pt (cl-transforms:make-quaternion 0 0 0 1)))
              (curry #'cl-transforms:v+ ref))
     (cons
      ref
      (funcall
       generator
       (cl-transforms:make-3d-vector
        (cl-transforms:x (bounding-box-dimensions aabb-bottom))
        (cl-transforms:y (bounding-box-dimensions aabb-bottom))
        0))))))

(defun obj-pose-on (pose obj)
  "Returns a new pose for `obj' such that the bottom of obj's bounding
  box is in `pose'"
  (let ((pose (ensure-pose pose)))
    (let* ((aabb-obj (aabb obj))
           (bounding-box-relative-offset
             (cl-transforms:v-
              (cl-transforms:origin (pose obj))
              (bounding-box-center aabb-obj)
              ;; The on relation implies that the bottom coordinate of
              ;; the object and pose should be aligned. x and y is
              ;; aligned by just using the object calculated so far,
              ;; but we need to add half of the z size.
              (cl-transforms:make-3d-vector
               0 0 (- (/ (cl-transforms:z (bounding-box-dimensions aabb-obj)) 2))))))
      (cl-transforms:copy-pose
       pose :origin (cl-transforms:v+
                     (cl-transforms:origin pose)
                     bounding-box-relative-offset)))))

(defun obj-poses-on (obj-name poses &optional (world *current-bullet-world*))
  "Returns a new lazy-list of poses for the object `obj' that are on
  the corresponding poses in `poses`."
  (let ((obj (object world obj-name)))
    (lazy-mapcar (lambda (pose)
                   (obj-pose-on
                    (etypecase pose
                      (cl-transforms:3d-vector (cl-transforms:make-pose
                                                pose
                                                (cl-transforms:make-quaternion 0 0 0 1)))
                      (cl-transforms:pose pose)
                      (list pose))
                    obj))
                 poses)))
