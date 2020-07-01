;;;
;;; Copyright (c) 2020, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :objects)

(defmethod man-int:get-action-gripping-effort :heuristics 20 ((type (eql :shoe)))
  30)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-action-gripper-opening :heuristics 20 ((type (eql :shoe)))
  0.1)

;;;;;;;;;;;;;;;;;;;;;;;;;;;; SHOE ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-object-type-to-gripper-transform ((object-type (eql :shoe))
                                                         object-name
                                                         arm
                                                         (grasp (eql :top)))
  (cl-transforms-stamped:make-transform-stamped
   (roslisp-utilities:rosify-underscores-lisp-name object-name)
   (ecase arm
     (:left cram-tf:*robot-left-tool-frame*)
     (:right cram-tf:*robot-right-tool-frame*))
   0.0
   (cl-transforms:make-3d-vector 0.0 0.0 0.0)
   (cl-transforms:make-quaternion 1 0 0 0)))

(defmethod man-int:get-object-type-to-gripper-pregrasp-transforms ((type (eql :shoe))
                                                                   object-name
                                                                   arm
                                                                   (grasp (eql :top))
                                                                   location
                                                                   grasp-transform)
  (list
   (cl-transforms-stamped:make-transform-stamped
    (roslisp-utilities:rosify-underscores-lisp-name object-name)
    (ecase arm
      (:left cram-tf:*robot-left-tool-frame*)
      (:right cram-tf:*robot-right-tool-frame*))
    0.0
    (cl-transforms:make-3d-vector 0.0 0.0 0.3)
    (cl-transforms:make-quaternion 1 0 0 0))
   (cl-transforms-stamped:make-transform-stamped
    (roslisp-utilities:rosify-underscores-lisp-name object-name)
    (ecase arm
      (:left cram-tf:*robot-left-tool-frame*)
      (:right cram-tf:*robot-right-tool-frame*))
    0.0
    (cl-transforms:make-3d-vector 0.0 0.0 0.2)
    (cl-transforms:make-quaternion 1 0 0 0))
   (cl-transforms-stamped:make-transform-stamped
    (roslisp-utilities:rosify-underscores-lisp-name object-name)
    (ecase arm
      (:left cram-tf:*robot-left-tool-frame*)
      (:right cram-tf:*robot-right-tool-frame*))
    0.0
    (cl-transforms:make-3d-vector 0.0 0.0 0.1)
    (cl-transforms:make-quaternion 1 0 0 0))))

(defmethod man-int:get-object-type-wrt-base-frame-lift-transforms ((type (eql :shoe))
                                                                   arm
                                                                   (grasp (eql :top))
                                                                   location)
  (list
   (cl-transforms-stamped:make-transform-stamped
    cram-tf:*robot-base-frame*
    cram-tf:*robot-base-frame*
    0.0
    (cl-transforms:make-3d-vector 0.0 0.0 0.1)
    (cl-transforms:make-identity-rotation))
   (cl-transforms-stamped:make-transform-stamped
    cram-tf:*robot-base-frame*
    cram-tf:*robot-base-frame*
    0.0
    (cl-transforms:make-3d-vector 0.0 0.0 0.2)
    (cl-transforms:make-identity-rotation))
   (cl-transforms-stamped:make-transform-stamped
    cram-tf:*robot-base-frame*
    cram-tf:*robot-base-frame*
    0.0
    (cl-transforms:make-3d-vector 0.0 0.0 0.3)
    (cl-transforms:make-identity-rotation))))
