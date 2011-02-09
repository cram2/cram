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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(in-package :location-costmap)

(defun 2d-cov (cov)
  (let ((result (make-array '(2 2) :element-type (array-element-type cov))))
    (declare (type (simple-array * 2) cov)
             (type (simple-array * (2 2)) result))
    (dotimes (y 2)
      (dotimes (x 2)
        (setf (aref result y x) (aref cov y x))))
    result))

(defun points-mean (pts)
  "Returns the mean vector of all points in `pts'"
  (let ((x 0.0d0)
        (y 0.0d0)
        (z 0.0d0)
        (n 0))
    (map 'nil (lambda (pt)
                (incf n)
                (incf x (cl-transforms:x pt))
                (incf y (cl-transforms:y pt))
                (incf z (cl-transforms:z pt)))
         pts)
    (cl-transforms:make-3d-vector (/ x n)
                                  (/ y n)
                                  (/ z n))))

(defun points-cov (pts &optional (mean (points-mean pts)))
  "Returns the covariance of `pts'."
  (let ((cov (make-array
              '(3 3)
              :element-type 'double-float
              :initial-element 0.0d0))
        (n 0)
        (accessors (make-array 3 :initial-contents (list #'cl-transforms:x
                                                         #'cl-transforms:y
                                                         #'cl-transforms:z))))
    (declare (type (simple-array double-float (3 3)) cov)
             (type (simple-array t (3)) accessors))
    (map 'nil (lambda (pt)
                (dotimes (y 3)
                  (dotimes (x (+ y 1))
                    (let ((val (* (- (funcall (aref accessors x) pt)
                                     (funcall (aref accessors x) mean))
                                  (- (funcall (aref accessors y) pt)
                                     (funcall (aref accessors y) mean)))))
                      (incf (aref cov y x) val))))
                (incf n))
         pts)
    (dotimes (y 3)
      (dotimes (x (+ 1 y))
        (setf (aref cov y x) (/ (aref cov y x) n))
        (unless (eql x y)
          (setf (aref cov x y) (aref cov y x)))))
    cov))
