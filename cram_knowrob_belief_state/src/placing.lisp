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

(in-package :kr-belief)

(defparameter *z-of-chassis-holder* 0.2 "In meters.")
(defparameter *x-of-chassis-holder-hole* 0.042 "In meters.")

(defgeneric get-object-placement-transform (on-object-type object-type arm grasp)
  (:documentation "Returns a transform from on-object to gripper -- ooTg."))

(defun get-object-type-put-pose (on-object-type object-type arm grasp on-object-transform)
  (declare (type cl-transforms-stamped:transform-stamped on-object-transform))
  "Returns a pose stamped representing bTg -- transfrom from base to gripper.

Take on-object-transform, ensure it's from base frame  -- bToo.
Multiply from the right with the transform from on-object to gripper -- bToo * ooTg == bTg."

  (unless (equal (cl-transforms-stamped:frame-id on-object-transform)
                   cram-tf:*robot-base-frame*)
      (error "In grasp calculations the ON-OBJECT-TRANSFORM did not have ~
correct parent frame: ~a and ~a"
             (cl-transforms-stamped:frame-id on-object-transform)
             cram-tf:*robot-base-frame*))

  (let ((on-object-gripper-transform
          (get-object-placement-transform on-object-type object-type arm grasp)))
    (cl-transforms-stamped:pose->pose-stamped
     cram-tf:*robot-base-frame*
     0.0
     (cl-transforms:transform->pose
      (cl-transforms:transform* on-object-transform on-object-gripper-transform)))))

(defun get-object-placing-poses (on-object-type object-type arm grasp on-object-transform)
  "Returns a list of (pregrasp-pose 2nd-pregrasp-pose grasp-pose lift-pose)"
  (let ((put-pose (get-object-type-put-pose on-object-type object-type arm grasp
                                            on-object-transform)))
    (list (get-object-type-lift-pose object-type arm grasp put-pose)
          put-pose
          (get-object-type-2nd-pregrasp-pose object-type arm grasp put-pose)
          (get-object-type-pregrasp-pose object-type arm grasp put-pose))))

(defmethod get-object-placement-transform ((on-object-type (eql :chassis-holder))
                                           (object-type (eql :chassis))
                                           (arm (eql :left))
                                           (grasp (eql :side)))
  (cl-transforms-stamped:make-transform-stamped
   "object_type_Chassis"
   "object_type_ChassisHolder"
   0.0
   (cl-transforms:make-3d-vector 0.0d0 0.0d0 *z-of-chassis-holder*)
   (cl-transforms:matrix->quaternion
    #2A((0 1 0)
        (0 0 -1)
        (-1 0 0)))))

(defmethod get-object-placement-transform ((on-object-type (eql :chassis))
                                           (object-type (eql :axle))
                                           (arm (eql :left))
                                           (grasp (eql :top)))
  (cl-transforms-stamped:make-transform-stamped
   "object_type_Chassis"
   "left_gripper_tool_frame"
   0.0
   (cl-transforms:make-3d-vector *x-of-chassis-holder-hole* 0 0)
   (cl-transforms:matrix->quaternion
    #2A((-1 0 0)
        (0 1 0)
        (0 0 -1)))))

(defmethod get-object-placement-transform ((on-object-type (eql :chassis))
                                           (object-type (eql :camaro-body))
                                           (arm (eql :left))
                                           (grasp (eql :top)))
  (cl-transforms-stamped:make-transform-stamped
   "object_type_Chassis"
   "left_gripper_tool_frame"
   0.0
   (cl-transforms:make-3d-vector 0 0 0)
   (cl-transforms:matrix->quaternion
    #2A((0 1 0)
        (-1 0 0)
        (0 0 1)))))
