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

(defun merge-bounding-boxes (box-1 box-2)
  (let* ((box-1/2 (cl-transforms:v*
                   (bounding-box-dimensions box-1)
                   0.5))
         (box-2/2 (cl-transforms:v*
                   (bounding-box-dimensions box-2)
                   0.5))
         (min-box-1 (cl-transforms:v- (bounding-box-center box-1) box-1/2))
         (max-box-1 (cl-transforms:v+ (bounding-box-center box-1) box-1/2))
         (min-box-2 (cl-transforms:v- (bounding-box-center box-2) box-2/2))
         (max-box-2 (cl-transforms:v+ (bounding-box-center box-2) box-2/2))
         (new-min (cl-transforms:make-3d-vector
                   (min (cl-transforms:x min-box-1)
                        (cl-transforms:x min-box-2))
                   (min (cl-transforms:y min-box-1)
                        (cl-transforms:y min-box-2))
                   (min (cl-transforms:z min-box-1)
                        (cl-transforms:z min-box-2))))
         (new-max (cl-transforms:make-3d-vector
                   (max (cl-transforms:x max-box-1)
                        (cl-transforms:x max-box-2))
                   (max (cl-transforms:y max-box-1)
                        (cl-transforms:y max-box-2))
                   (max (cl-transforms:z max-box-1)
                        (cl-transforms:z max-box-2)))))
    (make-bounding-box
     :center (cl-transforms:v* (cl-transforms:v+ new-max new-min)
                               0.5)
     :dimensions (cl-transforms:v- new-max new-min))))

(defmethod aabb ((obj object))
  (reduce #'merge-bounding-boxes
          (mapcar #'aabb (rigid-bodies obj))))

(defun calculate-object-bottom-pose (object)
  (let ((bounding-box (aabb object)))
    (cl-transforms:copy-pose
     (pose object)
     :origin (cl-transforms:copy-3d-vector
              (cl-transforms:origin (pose object))
              :z (- (cl-transforms:z (bounding-box-center bounding-box))
                    (/ (cl-transforms:z
                        (bounding-box-dimensions bounding-box)) 2))))))

(defun object-axis-facing-upwards (bullet-object)
  "Returns the axis `max-i' showing upwards and the direction which
can be up (1), down (-1) or inbetween by returning a value
`real-max-v' in [-1, 1]. `max-i' is 0 for the x-axis, 1 for the
y-axis or 2 for z-axis."
  (let ((max-i 0)
        (max-v most-negative-single-float)
        (real-max-v most-negative-single-float)
        (orientation-m (cl-transforms:quaternion->matrix
                        (cl-transforms:orientation 
                         (btr:pose
                          bullet-object)))))
    (loop for i from 0 to (1- (second (array-dimensions
                                       orientation-m))) 
          do
             (when (> (abs (aref orientation-m 2 i)) max-v)
               (setf max-v (abs (aref orientation-m 2 i)))
               (setf real-max-v (aref orientation-m 2 i))
               (setf max-i i)))
    (cons max-i real-max-v)))

(defun stabilized-identity-object-orientation (bullet-object)
  "Stabilized intend that only rotations around the z-axis in the object
frame are allowed (like objects laying on other horizontal objects).
Identity means that there are only 6 possible identity orientations of
objects represented in `different-axis-upwards'. This method takes a
`bullet-object' and returns a rotation from `different-axis-upwards'." 

  (let* ((z-up (cl-transforms:make-identity-rotation))
         (z-down (cl-transforms:make-quaternion 0 1 0 0))
         (x-up (cl-transforms:matrix->quaternion (make-array '(3 3)
                                                             :initial-contents
                                                             '((0  0 -1)
                                                               (0  1  0)
                                                               (1  0  0)))))
         (x-down (cl-transforms:q-inv x-up))
         (y-up (cl-transforms:matrix->quaternion (make-array '(3 3)
                                                             :initial-contents
                                                             '((1  0  0)
                                                               (0  0 -1)
                                                               (0  1  0)))))
         (y-down (cl-transforms:q-inv y-up))
         (different-axis-upwards `(,(list x-up x-down) 
                                   ,(list y-up y-down)
                                   ,(list z-up z-down)))
         (axis-facing-up-and-value (object-axis-facing-upwards bullet-object))
         (axis-facing-up (car axis-facing-up-and-value))
         (axis-facing-down-p (< (cdr axis-facing-up-and-value) 0)))

    (funcall (lambda (list)
               (if (not axis-facing-down-p)
                   (first list)
                   (second list)))
             (nth axis-facing-up different-axis-upwards))))
  

(defun calculate-bb-dims (bullet-object)
  (let ((old-pose (pose bullet-object))
        aabb)
    (unwind-protect
         (progn
           (setf (pose bullet-object)
                 (cl-transforms:make-pose
                  (cl-transforms:make-identity-vector)
                  (stabilized-identity-object-orientation bullet-object)))
           (setf aabb (cl-bullet:aabb bullet-object)))
      (setf (pose bullet-object) old-pose))
    (cl-bullet:bounding-box-dimensions aabb)))
