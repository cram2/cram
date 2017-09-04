;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :pr2-cloud)

(defun calculate-robust-handle-and-joint-transform (handle-transform joint-transform)
  ;; (return-from calculate-robust-handle-and-joint-transform
  ;;   (values handle-transform joint-transform))
  (let* ((handle->joint-vector
           (cl-transforms:v-
            (cl-transforms-stamped:translation joint-transform)
            (cl-transforms-stamped:translation handle-transform)))
         (handle->joint-vector-projected-onto-xy
           (cl-transforms:copy-3d-vector handle->joint-vector :z 0))
         (y-axis handle->joint-vector-projected-onto-xy)
         (z-axis (cl-transforms:make-3d-vector 0 0 1))
         (x-axis (cl-transforms:cross-product y-axis z-axis))
         (orientation (cl-transforms:column-vectors->quaternion x-axis y-axis z-axis)))
    (values
     (copy-transform-stamped
      handle-transform
      :rotation orientation)
     (copy-transform-stamped
      joint-transform
      :rotation orientation))))

(defun calculate-handle-to-gripper-transforms (map-to-handle map-to-joint
                                               &optional (theta-max
                                                          (cma:degrees->radians 60)))
  (let* ((joint-to-handle
           (apply-transform (cram-tf:transform-stamped-inv map-to-joint)
                            map-to-handle))
         (handle-to-joint (cram-tf:transform-stamped-inv joint-to-handle)))
    (mapcar (lambda (joint-to-circle-point)
              (pose-stamped->transform-stamped
               (cram-tf:rotate-pose
                (strip-transform-stamped
                 (apply-transform handle-to-joint joint-to-circle-point))
                :z
                (cram-math:degrees->radians 180))
               (cl-transforms-stamped:child-frame-id joint-to-circle-point)))
            (loop for theta = 0.0 then (+ 0.1 theta)
                  while (< theta theta-max)
                  collect
                  (let ((rotation
                          (cl-tf:axis-angle->quaternion
                           (cl-transforms:make-3d-vector 0 0 1) theta)))
                    (cl-transforms-stamped:make-transform-stamped
                     (cl-transforms-stamped:frame-id joint-to-handle)
                     cram-tf:*robot-right-tool-frame*
                     (cl-transforms-stamped:stamp joint-to-handle)
                     (cl-transforms:rotate rotation (cl-transforms:translation joint-to-handle))
                     (cl-transforms:rotation joint-to-handle)))))))

(defun filter-trajectory-of-big-rotations (transforms-list &optional (rotation-threshold 0.1))
  (remove NIL
          (remove-duplicates
           (maplist (lambda (x)
                      (when (>= (length x) 2)
                        (let* ((quaternion-diff
                                 (cl-transforms:quaternion->axis-angle
                                  (cl-transforms:q- (cl-transforms:rotation (second x))
                                                    (cl-transforms:rotation (first x)))))
                               (max-val
                                 (with-slots (cl-transforms:x cl-transforms:y cl-transforms:z)
                                     quaternion-diff
                                   (max (abs cl-transforms:x)
                                        (abs cl-transforms:y)
                                        (abs cl-transforms:z)))))
                          (if (< max-val rotation-threshold)
                              (first x)
                              NIL))))
                    transforms-list))))
