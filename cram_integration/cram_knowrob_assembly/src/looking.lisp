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

(in-package :kr-assembly)

(defparameter *left-arm-out-of-field-of-view-state*
  '(-1.858d0 0.70571d0 0.9614d0 -0.602d0 -2.5922d0 -1.94065d0 -1.28735d0))
(defparameter *right-arm-out-of-field-of-view-state*
  '(0.0 0.0 0.0 0.0 0.0 0.0 0.0))

(defparameter *left-arm-nicer-configuration*
  '(-1.2274070978164673 0.8496202230453491 -0.10349386930465698 -1.0852965116500854 -0.4587952196598053 1.259474515914917 -0.06962397694587708))

(defparameter *default-look-z-offset* 0.2 "in meters")
(defparameter *default-look-x-offset* 0.15 "in meters")

(defun get-object-look-pose (arm object-transform)
  (declare (type cl-transforms-stamped:transform-stamped object-transform))
  "Returns a pose stamped representing bTg -- transfrom from base to gripper.

Take object-transform, ensure it's from base frame  -- bTo.
Take an initial pose of gripper in base bTg and set its X and Y to object X and Y.
Set Z to offset object Z."

  (unless (equal (cl-transforms-stamped:frame-id object-transform)
                 cram-tf:*robot-base-frame*)
    (error "In grasp calculations the OBJECT-TRANSFORM did not have ~
correct parent frame: ~a and ~a"
           (cl-transforms-stamped:frame-id object-transform)
           cram-tf:*robot-base-frame*))

  (if (eql arm :left)
      (let* ((gripper-initial-pose
               (cl-transforms-stamped:make-pose-stamped
                cram-tf:*robot-base-frame*
                0.0
                (cl-transforms:make-3d-vector 0 0 0)
                (cl-transforms:matrix->quaternion
                 #2A((-1 0 0)
                     (0 1 0)
                     (0 0 -1)))))
             (object-x-in-base (cl-transforms:x (cl-transforms:translation object-transform)))
             (object-y-in-base (cl-transforms:y (cl-transforms:translation object-transform)))
             (object-z-in-base (cl-transforms:z (cl-transforms:translation object-transform)))
             (offset-object-x (- object-x-in-base *default-look-x-offset*))
             (offset-object-z (+ object-z-in-base *default-look-z-offset*)))
        (cl-transforms-stamped:copy-pose-stamped
         gripper-initial-pose
         :origin (cl-transforms:make-3d-vector offset-object-x object-y-in-base offset-object-z)))
      NIL))
