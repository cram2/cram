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

(defparameter *default-z-offset* 0.2 "in meters")
(defparameter *default-small-z-offset* 0.07 "in meters")

(defgeneric get-object-type-grasp (object-type)
  (:documentation "Returns either of :top, :side, :front. Default is :top.")
  (:method (object-type) :top)
  (:method ((object-type (eql :porsche-body))) :top)
  (:method ((object-type (eql :camaro-body))) :top)
  (:method ((object-type (eql :chassis))) :side)
  (:method ((object-type (eql :axle))) :top)
  (:method ((object-type (eql :wheel))) :top))

(defgeneric get-object-type-gripping-effort (object-type)
  (:documentation "Returns effort in Nm, e.g. 50. Default is 35.")
  (:method (object-type) 35)
  (:method ((object-type (eql :porsche-body))) 35)
  (:method ((object-type (eql :camaro-body))) 35)
  (:method ((object-type (eql :chassis))) 50)
  (:method ((object-type (eql :axle))) 50)
  (:method ((object-type (eql :wheel))) 40))

(defgeneric get-object-type-gripper-opening (object-type)
  (:documentation "How wide to open the gripper before grasping, in m. Default is 0.10.")
  (:method (object-type) 0.10)
  (:method ((object-type (eql :axle))) 0.02))

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

  (unless (equal (cl-transforms-stamped:frame-id x-y-transform) x-frame)
      (error "In multiply-transform-stampeds X-Y-TRANSFORM did not have ~
              correct parent frame: ~a and ~a"
             (cl-transforms-stamped:frame-id x-y-transform) x-frame))

  (unless (equal (cl-transforms-stamped:child-frame-id y-z-transform) z-frame)
      (error "In multiply-transform-stampeds Y-Z-TRANSFORM did not have ~
              correct child frame: ~a and ~a"
             (cl-transforms-stamped:child-frame-id y-z-transform) z-frame))

  (unless (equal (cl-transforms-stamped:child-frame-id x-y-transform)
                 (cl-transforms-stamped:frame-id y-z-transform))
      (error "In multiply-transform-stampeds X-Y-TRANSFORM and ~
              Y-Z-TRANSFORM did not have equal corresponding frames: ~a and ~a"
             (cl-transforms-stamped:child-frame-id x-y-transform)
             (cl-transforms-stamped:frame-id y-z-transform)))

  (let ((multiplied-transforms
          (cl-transforms:transform* x-y-transform y-z-transform)))
    (ecase result-as-pose-or-transform
      (:pose
       (cl-transforms-stamped:pose->pose-stamped
        x-frame
        0.0
        (cl-transforms:transform->pose multiplied-transforms)))
      (:transform
       (cl-transforms-stamped:transform->transform-stamped
        x-frame
        z-frame
        0.0
        multiplied-transforms)))))

;; (defun get-object-grasping-poses (object-name arm grasp object-transform)
;;   "Returns a list of (pregrasp-pose 2nd-pregrasp-pose grasp-pose lift-pose)"
;;   (mapcar (lambda (manipulation-type)
;;             (get-gripper-in-base-pose
;;              arm object-transform ; bTo
;;              (get-object-manipulation-transform ; gTo
;;               manipulation-type "left_gripper" object-name grasp)
;;              cram-tf:*robot-left-tool-frame*)) ; bTo * oTg = bTg
;;           '(:pregrasp :pregrasp :grasp :lift)))
;; :pregrasp is two times because for some objects there should
;; actually be :2nd-pregrasp and that is what the plans expect

(defgeneric get-object-type-pregrasp-pose (object-type arm grasp grasp-pose)
  (:documentation "Returns a pose stamped"))

(defgeneric get-object-type-2nd-pregrasp-pose (object-type arm grasp grasp-pose)
  (:documentation "Returns a pose stamped. Default value is NIL.")
  (:method (object-type grasp-pose arm grasp) nil))

(defgeneric get-object-type-lift-pose (object-type arm grasp grasp-pose)
  (:documentation "Returns a pose stamped"))

(defgeneric get-object-type-2nd-lift-pose (object-type arm grasp grasp-pose)
  (:documentation "Returns a pose stamped")
  (:method (object-type grasp-pose arm grasp) nil))

(defun get-object-grasping-poses (object-name object-type arm grasp object-transform)
  "Returns a list of (pregrasp-pose 2nd-pregrasp-pose grasp-pose lift-pose)"
  (let ((grasp-pose (multiply-transform-stampeds
                     cram-tf:*robot-base-frame* cram-tf:*robot-left-tool-frame*
                     object-transform ; bTo
                     (transform-stamped-inv
                      (get-object-manipulation-transform ; gTo
                       :grasp "left_gripper" object-name grasp)) ; oTg
                     :result-as-pose-or-transform :pose))) ; bTo * oTg = bTg
    (list (get-object-type-pregrasp-pose object-type arm grasp grasp-pose)
          ;; (get-object-type-2nd-pregrasp-pose object-type arm grasp grasp-pose)
          grasp-pose
          (get-object-type-lift-pose object-type arm grasp grasp-pose))))


(defmethod get-object-type-pregrasp-pose ((object-type (eql :axle))
                                          (arm (eql :left))
                                          (grasp (eql :top))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose :z-offset *default-z-offset*))
(defmethod get-object-type-lift-pose ((object-type (eql :axle))
                                      (arm (eql :left))
                                      (grasp (eql :top))
                                      grasp-pose)
  (cram-tf:translate-pose grasp-pose :z-offset *default-z-offset*))
(defmethod get-object-type-2nd-lift-pose ((object-type (eql :axle))
                                          (arm (eql :left))
                                          (grasp (eql :top))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose :z-offset *default-small-z-offset*))

(defmethod get-object-type-pregrasp-pose ((object-type (eql :chassis))
                                          (arm (eql :left))
                                          (grasp (eql :side))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose :y-offset *default-z-offset*))
(defmethod get-object-type-lift-pose ((object-type (eql :chassis))
                                      (arm (eql :left))
                                      (grasp (eql :side))
                                      grasp-pose)
  (cram-tf:translate-pose grasp-pose :z-offset *default-z-offset*)
  ;; (cl-transforms-stamped:copy-pose-stamped
  ;;  (cram-tf:translate-pose grasp-pose :z-offset 0.30)
  ;;  :orientation
  ;;  (cl-transforms:matrix->quaternion
  ;;   #2A((-1 0 0)
  ;;       (0 0 -1)
  ;;       (0 -1 0))))
  )
(defmethod get-object-type-2nd-lift-pose ((object-type (eql :chassis))
                                          (arm (eql :left))
                                          (grasp (eql :side))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose :z-offset *default-small-z-offset*))

(defmethod get-object-type-pregrasp-pose ((object-type (eql :camaro-body))
                                          (arm (eql :left))
                                          (grasp (eql :top))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose :z-offset *default-z-offset*))
(defmethod get-object-type-lift-pose ((object-type (eql :camaro-body))
                                      (arm (eql :left))
                                      (grasp (eql :top))
                                      grasp-pose)
  (cram-tf:translate-pose grasp-pose :z-offset *default-z-offset*))
(defmethod get-object-type-2nd-lift-pose ((object-type (eql :camaro-body))
                                          (arm (eql :left))
                                          (grasp (eql :top))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose :z-offset *default-small-z-offset*))


#+everything-below-is-commented-out
(
(defmethod get-object-to-gripper-transform ((object-type (eql :axle))
                                            (arm (eql :left))
                                            (grasp (eql :top)))
  (cl-transforms-stamped:make-transform-stamped
   "object_of_type_Axle"
   "left_gripper_tool_frame"
   0.0
   (cl-transforms:make-3d-vector 0.0d0 0.0d0 0.0d0)
   (cl-transforms:matrix->quaternion
    #2A((-1 0 0)
        (0 1 0)
        (0 0 -1)))))

(defmethod get-object-to-gripper-transform ((object-type (eql :chassis))
                                            (arm (eql :left))
                                            (grasp (eql :side)))
  (cl-transforms-stamped:make-transform-stamped
   "object_of_type_Chassis"
   "left_gripper_tool_frame"
   0.0
   (cl-transforms:make-3d-vector 0.0d0 0.0d0 0.0d0)
   (cl-transforms:matrix->quaternion
    #2A((0 -1 0)
        (0 0 -1)
        (1 0 0)))))

(defmethod get-object-to-gripper-transform ((object-type (eql :camaro-body))
                                            (arm (eql :left))
                                            (grasp (eql :top)))
  (cl-transforms-stamped:make-transform-stamped
   "object_of_type_CamaroBody"
   "left_gripper_tool_frame"
   0.0
   (cl-transforms:make-3d-vector 0.0d0 0.0d0 0.0d0)
   (cl-transforms:matrix->quaternion
    #2A((0 1 0)
        (1 0 0)
        (0 0 -1)))))
)
