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

(defun pose->list (pose)
  (let* ((xyz (cl-transforms:origin pose))
         (qqqw (cl-transforms:orientation pose))
         (x (cl-transforms:x xyz))
         (y (cl-transforms:y xyz))
         (z (cl-transforms:z xyz))
         (q1 (cl-transforms:x qqqw))
         (q2 (cl-transforms:y qqqw))
         (q3 (cl-transforms:z qqqw))
         (w (cl-transforms:w qqqw)))
    `((,x ,y ,z) (,q1 ,q2 ,q3 ,w))))

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

(defun list->pose (pose-list)
  (destructuring-bind ((x y z) (q1 q2 q3 w))
      pose-list
    (cl-transforms:make-pose
     (cl-transforms:make-3d-vector x y z)
     (cl-transforms:make-quaternion q1 q2 q3 w))))

(defun ensure-pose-in-frame (pose frame &key use-current-ros-time use-zero-time)
  (declare (type (or null cl-transforms:pose cl-transforms-stamped:pose-stamped)))
  (when pose
    (cl-transforms-stamped:transform-pose-stamped
     *transformer*
     :pose (cl-transforms-stamped:ensure-pose-stamped
            (if use-zero-time
                (cl-transforms-stamped:copy-pose-stamped pose :stamp 0.0)
                pose)
            frame
            0.0)
     :target-frame frame
     :timeout *tf-default-timeout*
     :use-current-ros-time use-current-ros-time)))

(defun ensure-point-in-frame (point frame &key use-current-ros-time use-zero-time)
  (declare (type (or cl-transforms:point cl-transforms-stamped:point-stamped)))
  (cl-transforms-stamped:transform-point-stamped
   *transformer*
   :point (if (typep point 'cl-transforms-stamped:point-stamped)
              (if use-zero-time
                  (with-slots (frame-id origin) point
                    (cl-transforms-stamped:make-point-stamped frame-id 0.0 origin))
                  point)
              (cl-transforms-stamped:make-point-stamped
               frame 0.0 point))
   :target-frame frame
   :timeout *tf-default-timeout*
   :use-current-ros-time use-current-ros-time))

(defun translate-pose (pose &key (x-offset 0.0) (y-offset 0.0) (z-offset 0.0))
  (let* ((pose-origin
           (cl-transforms:origin pose))
         (new-origin
           (cl-transforms:copy-3d-vector
            pose-origin
            :x (let ((x-pose-origin (cl-transforms:x pose-origin)))
                 (+ x-pose-origin x-offset))
            :y (let ((y-pose-origin (cl-transforms:y pose-origin)))
                 (+ y-pose-origin y-offset))
            :z (let ((z-pose-origin (cl-transforms:z pose-origin)))
                 (+ z-pose-origin z-offset)))))
    (etypecase pose
      (cl-transforms-stamped:pose-stamped
       (cl-transforms-stamped:copy-pose-stamped pose :origin new-origin))
      (cl-transforms:pose
       (cl-transforms:copy-pose pose :origin new-origin)))))

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

(defun tf-frame-converged (goal-frame goal-pose-stamped delta-xy delta-theta)
  (let* ((pose-in-frame
           (cram-tf:ensure-pose-in-frame
            goal-pose-stamped
            goal-frame
            :use-zero-time t))
         (goal-dist (max (abs (cl-transforms:x (cl-transforms:origin pose-in-frame)))
                         (abs (cl-transforms:y (cl-transforms:origin pose-in-frame)))))
         (goal-angle (cl-transforms:normalize-angle
                      (cl-transforms:get-yaw
                       (cl-transforms:orientation pose-in-frame)))))
    (and (<= goal-dist delta-xy)
         (<= (abs goal-angle) delta-theta))))

(defun pose->transform-stamped (parent-frame child-frame stamp pose)
  (let ((translation (cl-transforms:origin pose))
        (rotation (cl-transforms:orientation pose)))
    (cl-transforms-stamped:make-transform-stamped
     parent-frame child-frame stamp translation rotation)))

(defun transform-stamped-inv (transform-stamped)
  (let ((frame-id (cl-transforms-stamped:frame-id transform-stamped))
        (child-frame-id (cl-transforms-stamped:child-frame-id transform-stamped))
        (stamp (cl-transforms-stamped:stamp transform-stamped)))
    (cl-transforms-stamped:transform->transform-stamped
     child-frame-id
     frame-id
     stamp
     (cl-transforms:transform-inv transform-stamped))))

(defun multiply-transform-stampeds (x-frame z-frame
                                    x-y-transform y-z-transform
                                    &key (result-as-pose-or-transform :transform))
  (declare (type cl-transforms-stamped:transform-stamped
                 x-y-transform y-z-transform)
           (type keyword result-as-pose-or-transform)
           (type string x-frame z-frame))
  "Returns a pose stamped representing xTz -- transfrom from x-frame to z-frame.

Take xTy, ensure it's from x-frame.
Multiply from the right with the yTz transform -- xTy * yTz == xTz."

  (unless (string-equal (cl-transforms-stamped:frame-id x-y-transform) x-frame)
    (warn "~%~%~%~%!!!!!~%~%~%In multiply-transform-stampeds X-Y-TRANSFORM did not have ~
              correct parent frame: ~a and ~a"
           (cl-transforms-stamped:frame-id x-y-transform) x-frame))

  (unless (string-equal (cl-transforms-stamped:child-frame-id y-z-transform) z-frame)
    (warn "~%~%~%~%!!!!!~%~%~%In multiply-transform-stampeds Y-Z-TRANSFORM did not have ~
              correct child frame: ~a and ~a"
           (cl-transforms-stamped:child-frame-id y-z-transform) z-frame))

  (unless (string-equal (cl-transforms-stamped:child-frame-id x-y-transform)
                        (cl-transforms-stamped:frame-id y-z-transform))
    (warn "~%~%~%~%!!!!!~%~%~%In multiply-transform-stampeds X-Y-TRANSFORM and ~
              Y-Z-TRANSFORM did not have equal corresponding frames: ~a and ~a"
           (cl-transforms-stamped:child-frame-id x-y-transform)
           (cl-transforms-stamped:frame-id y-z-transform)))

  (let ((multiplied-transforms
          (cl-transforms:transform* x-y-transform y-z-transform))
        (timestamp (min (cl-transforms-stamped:stamp x-y-transform)
                        (cl-transforms-stamped:stamp y-z-transform))))
    (ecase result-as-pose-or-transform
      (:pose
       (cl-transforms-stamped:pose->pose-stamped
        x-frame
        timestamp
        (cl-transforms:transform->pose multiplied-transforms)))
      (:transform
       (cl-transforms-stamped:transform->transform-stamped
        x-frame
        z-frame
        timestamp
        multiplied-transforms)))))

(defun strip-transform-stamped (transform-stamped)
  (cl-transforms-stamped:make-pose-stamped
   (cl-transforms-stamped:frame-id transform-stamped)
   (cl-transforms-stamped:stamp transform-stamped)
   (cl-transforms-stamped:translation transform-stamped)
   (cl-transforms:rotation transform-stamped)))

(defun copy-transform-stamped (transform-stamped &key frame-id child-frame-id stamp
                                                   translation rotation)
  (cl-transforms-stamped:make-transform-stamped
   (or frame-id (cl-transforms-stamped:frame-id transform-stamped))
   (or child-frame-id (cl-transforms-stamped:child-frame-id transform-stamped))
   (or stamp (cl-transforms-stamped:stamp transform-stamped))
   (or translation (cl-transforms-stamped:translation transform-stamped))
   (or rotation (cl-transforms-stamped:rotation transform-stamped))))

(defun translate-transform-stamped (transform &key (x-offset 0.0) (y-offset 0.0) (z-offset 0.0))
  (copy-transform-stamped
   transform
   :translation (let ((transform-translation (cl-transforms:translation transform)))
                  (cl-transforms:copy-3d-vector
                   transform-translation
                   :x (let ((x-transform-translation (cl-transforms:x transform-translation)))
                        (+ x-transform-translation x-offset))
                   :y (let ((y-transform-translation (cl-transforms:y transform-translation)))
                        (+ y-transform-translation y-offset))
                   :z (let ((z-transform-translation (cl-transforms:z transform-translation)))
                        (+ z-transform-translation z-offset))))))

(defun pose-stamped->transform-stamped (pose-stamped child-frame-id)
  (cl-transforms-stamped:make-transform-stamped
   (cl-transforms-stamped:frame-id pose-stamped)
   child-frame-id
   (cl-transforms-stamped:stamp pose-stamped)
   (cl-transforms-stamped:origin pose-stamped)
   (cl-transforms-stamped:orientation pose-stamped)))

(defun apply-transform (left-hand-side-transform right-hand-side-transform)
  (cram-tf:multiply-transform-stampeds
   (cl-transforms-stamped:frame-id left-hand-side-transform)
   (cl-transforms-stamped:child-frame-id right-hand-side-transform)
   left-hand-side-transform
   right-hand-side-transform))
