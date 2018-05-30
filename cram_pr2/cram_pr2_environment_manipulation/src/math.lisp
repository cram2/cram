;;;
;;; Copyright (c) 2018, Christopher Pollok <cpollok@uni-bremen.de>
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

(in-package :pr2-em)

(defun line-equation (x1 y1 x2 y2)
  "Return the a, b and c values of the equation descriping a straight line
through (x1,y1) and (x2,y2)."
  (let ((x (- x2 x1))
        (y (- y2 y1)))
    (if (eq 0 x)
        (values 0 x 0)
        (let ((m (/ y x)))
          (values
           (- 0 m)
           1
           (- 0 (- y1 (* m x1))))))))

(defun distance-point (a b c x y)
  "Return the coordinates on the line described by ax + bx + c
from which the distance to the point (x,y) is minimal."
  (values
   (/
    (- (* b (- (* b x) (* a y))) (* a c))
    (+ (expt a 2) (expt b 2)))
   (/
    (- (* a (+ (* (- 0 b) x) (* a y))) (* b c))
    (+ (expt a 2) (expt b 2)))))

(defun line-equation-in-xy (p1 p2)
  "Return the equation (a,b,c values) describing a line between p1 and p2 in the x-y-plane."
  (let ((x1 (cl-tf:x p1))
        (y1 (cl-tf:y p1))
        (x2 (cl-tf:x p2))
        (y2 (cl-tf:y p2)))
    (line-equation x1 y1 x2 y2)))

(defun line-p-dist (a b c p)
  "Return the disctance between the line described by ax + bx + c
and the point p (in the x-y plane)."
  (let ((x (cl-tf:x p))
        (y (cl-tf:y p)))
    (/
     (abs (+ (* a x) (* b y) c))
     (sqrt (+ (expt a 2) (expt b 2))))))

(defun line-p-dist-point (a b c p)
  "Return the point on the line described by ax + bx + c
from which the distance to the point (x,y) is minimal."
  (let ((px (cl-tf:x p))
        (py (cl-tf:y p)))
    (multiple-value-bind (x y)
        (distance-point a b c px py)
      (cl-tf:make-3d-vector x y 0))))
