;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :cram-tf)

(defun poses-equal-p (pose-1 pose-2 dist-sigma ang-sigma)
  (and (<= (cl-transforms:v-dist (cl-transforms:origin pose-1)
                                 (cl-transforms:origin pose-2))
           dist-sigma)
       (<= (cl-transforms:angle-between-quaternions
            (cl-transforms:orientation pose-1)
            (cl-transforms:orientation pose-2))
           ang-sigma)))

(defun frame-to-pose-in-fixed-frame (frame-name)
  (when *transformer*
    (transform-pose-stamped
     *transformer*
     :timeout *tf-default-timeout*
     :pose (make-pose-stamped frame-name 0.0
                              (make-identity-vector)
                              (make-identity-rotation))
     :target-frame *fixed-frame*)))

(defun pose->flat-list (pose)
  (let* ((xyz (cl-transforms:origin pose))
         (qqqw (cl-transforms:orientation pose))
         (x (cl-transforms:x xyz))
         (y (cl-transforms:y xyz))
         (z (cl-transforms:z xyz))
         (q1 (cl-transforms:x qqqw))
         (q2 (cl-transforms:y qqqw))
         (q3 (cl-transforms:z qqqw))
         (w (cl-transforms:w qqqw)))
    (list x y z q1 q2 q3 w)))

(defun pose->flat-list-w-first (pose)
  (let* ((xyz (cl-transforms:origin pose))
         (qqqw (cl-transforms:orientation pose))
         (x (cl-transforms:x xyz))
         (y (cl-transforms:y xyz))
         (z (cl-transforms:z xyz))
         (q1 (cl-transforms:x qqqw))
         (q2 (cl-transforms:y qqqw))
         (q3 (cl-transforms:z qqqw))
         (w (cl-transforms:w qqqw)))
    (list x y z w q1 q2 q3)))

(defun flat-list->pose (pose-list)
  (destructuring-bind (x y z q1 q2 q3 w)
      pose-list
    (cl-transforms:make-pose
     (cl-transforms:make-3d-vector x y z)
     (cl-transforms:make-quaternion q1 q2 q3 w))))

(defun flat-list-w-first->pose (pose-list)
  (destructuring-bind (x y z w q1 q2 q3)
      pose-list
    (cl-transforms:make-pose
     (cl-transforms:make-3d-vector x y z)
     (cl-transforms:make-quaternion q1 q2 q3 w))))

(defun ensure-pose-in-frame (pose frame)
  (declare (type (or null cl-transforms:pose cl-transforms-stamped:pose-stamped)))
  (when pose
    (cl-transforms-stamped:transform-pose-stamped
     *transformer*
     :pose (cl-transforms-stamped:ensure-pose-stamped pose frame 0.0)
     :target-frame frame
     :timeout *tf-default-timeout*
     :use-current-ros-time t)))

(defun ensure-point-in-frame (point frame)
  (declare (type (or cl-transforms:point cl-transforms-stamped:point-stamped)))
  (cl-transforms-stamped:transform-point-stamped
   *transformer*
   :point (if (typep point 'cl-transforms-stamped:point-stamped)
              point
              (cl-transforms-stamped:make-point-stamped
               frame 0.0 point))
   :target-frame frame
   :timeout *tf-default-timeout*))

(defun translate-pose (pose &key (x-offset 0.0) (y-offset 0.0) (z-offset 0.0))
  (cl-transforms-stamped:copy-pose-stamped
   pose
   :origin (let ((pose-origin (cl-transforms:origin pose)))
             (cl-transforms:copy-3d-vector
              pose-origin
              :x (let ((x-pose-origin (cl-transforms:x pose-origin)))
                   (+ x-pose-origin x-offset))
              :y (let ((y-pose-origin (cl-transforms:y pose-origin)))
                   (+ y-pose-origin y-offset))
              :z (let ((z-pose-origin (cl-transforms:z pose-origin)))
                   (+ z-pose-origin z-offset))))))

(defun rotate-pose (pose axis angle)
  (cl-transforms-stamped:copy-pose-stamped
   pose
   :orientation (let ((pose-orientation (cl-transforms:orientation pose)))
                  (cl-transforms:q*
                   (cl-transforms:axis-angle->quaternion
                    (case axis
                      (:x (cl-transforms:make-3d-vector 1 0 0))
                      (:y (cl-transforms:make-3d-vector 0 1 0))
                      (:z (cl-transforms:make-3d-vector 0 0 1))
                      (t (error "[CRAM-TF:ROTATE-POSE] axis ~a not specified properly" axis)))
                    angle)
                   pose-orientation))))
