;;;
;;; Copyright (c) 2015, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(defun robot-current-pose (&key use-current-time-p)
  (if *transformer*
      (let* ((time (if use-current-time-p
                       (roslisp:ros-time)
                       0.0))
             (robot-pose (transform-pose-stamped
                          *transformer*
                          :target-frame *fixed-frame*
                          :pose (make-pose-stamped
                                 *robot-base-frame*
                                 time
                                 (cl-transforms:make-identity-vector)
                                 (cl-transforms:make-identity-rotation))
                          :timeout *tf-default-timeout*)))
        ;; NOTE(moesenle): Unfortunately, the robot's pose can be slightly
        ;; below (or maybe above) the floor. This can screw up designator
        ;; validation. To fix it for now, just set the z coordinate to
        ;; zero. This is an ugly hack that I feel bad about. Someone needs
        ;; to fix it in the future.
        (copy-pose-stamped
         robot-pose
         :origin (cl-transforms:copy-3d-vector
                  (cl-transforms:origin robot-pose)
                  :z 0.0)))
      (error 'transform-stamped-error
             :description "*transformer* is NIL. Have you called STARTUP-ROS?")))
