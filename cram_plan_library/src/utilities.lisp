;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :plan-lib)

(defun next-different-location-solution (designator &optional (threshold 0.05))
  "Returns a new designator solution that is at a different place than
  the current solution of `designator'."
  (declare (type location-designator designator))
  (designators-ros:next-filtered-designator-solution
   designator (designators-ros:make-euclidean-distance-filter
               (reference designator) threshold)))

(defun current-robot-location ()
  ;; NOTE(moesenle): Unfortunately, the robot's pose can be slightly
  ;; below (or maybe above) the floor. This can screw up designator
  ;; validation. To fix it for now, just set the z coordinate to
  ;; zero. This is an ugly hack that I feel bad about. Someone needs
  ;; to fix it in the future.
  (let ((robot-pose
          (cl-tf2:transform-pose
           *tf2-buffer*
           :pose (cl-tf-datatypes:make-pose-stamped
                  designators-ros:*robot-base-frame* 0.0
                  (cl-transforms:make-identity-vector)
                  (cl-transforms:make-identity-rotation))
           :target-frame designators-ros:*fixed-frame*
           :timeout cram-roslisp-common:*tf-default-timeout*)))
    (cl-tf-datatypes:copy-pose-stamped
     robot-pose
     :origin (cl-transforms:copy-3d-vector
              (cl-transforms:origin robot-pose) :z 0.0))))

(defun distance-to-drive (goal)
  (let ((loc-1 (reference goal))
        (current-loc (current-robot-location)))
    (cl-transforms:v-dist (cl-transforms:origin loc-1)
                          (cl-transforms:origin current-loc))))

(defmacro retry-with-updated-location (location update-form)
  "If the evaluation of `update-form' returns non-nil, sets `location'
  to the result of `update-form' and executes RETRY.

  This macro can only be used in the dynamic extent of
  WITH-FAILURE-HANDLING."
  `(let ((new-location ,update-form))
     (when new-location
       (setf ,location new-location)
       (retry))))
