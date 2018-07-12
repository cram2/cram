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

(defun calculate-bb-dims (bullet-object)
  (let ((old-pose (pose bullet-object))
        aabb)
    (unwind-protect
         (progn
           (setf (pose bullet-object) (cl-transforms:make-identity-pose))
           (setf aabb (cl-bullet:aabb bullet-object)))
      (setf (pose bullet-object) old-pose))
    (cl-bullet:bounding-box-dimensions aabb)))
