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

(defparameter *z-of-chassis-holder* 0.2 "In meters.")
(defparameter *x-of-chassis-holder-hole* 0.042 "In meters.")

;; (defun get-object-grasping-poses (on-object-name on-object-type
;;                                   object-name object-type arm grasp on-object-transform)
;;   "Returns a list of (pregrasp-pose 2nd-pregrasp-pose grasp-pose lift-pose)"
;;   (mapcar (lambda (manipulation-type)
;;             (get-gripper-in-base-pose
;;              arm on-object-transform ; bToo
;;              (get-object-manipulation-transform ; gToo aka oToo
;;               manipulation-type "left_gripper" object-name grasp))) ; bToo * ooTg = bTg
;;           '(:lift :connect :pregrasp :pregrasp)))

(defun get-object-placing-poses (on-object-name on-object-type object-name object-type
                                 arm grasp on-object-transform)
  "Returns a list of (pregrasp-pose 2nd-pregrasp-pose grasp-pose lift-pose)"
  (flet ((get-connection-id (object-name connect-to-object-name)
           (ecase object-name
             (:axle1 (ecase connect-to-object-name
                       (:chassis1 :axle-snap-in-front)))
             (:axle2 (ecase connect-to-object-name
                       (:chassis1 :axle-snap-in-back)))
             (:chassis1 (ecase connect-to-object-name
                          (:chassis-holder1 :chassis-on-holder)))
             (:camaro-body1 (ecase connect-to-object-name
                              (:chassis1 :chassis-snap-in-connection)))
             (:seat1 (ecase connect-to-object-name
                       (:chassis1 :seat-snap-in-front)))
             (:seat2 (ecase connect-to-object-name
                       (:chassis1 :seat-snap-in-back))))))

    (let* ((put-pose (cram-tf:multiply-transform-stampeds
                      cram-tf:*robot-base-frame* cram-tf:*robot-left-tool-frame*
                      (cram-tf:multiply-transform-stampeds
                       cram-tf:*robot-base-frame* (cram->knowrob object-name)
                       on-object-transform ; bToo
                       ;; (get-object-placement-transform ; oToo aka gToo
                       ;;  on-object-name object-name "left_gripper" grasp)))) ; bToo * ooTg = bTg
                       (get-object-connection-transform ; ooTo
                        (get-connection-id object-name on-object-name)
                        on-object-name
                        object-name)) ; bToo * ooTo = bTo
                      (cram-tf:transform-stamped-inv
                       (get-object-manipulation-transform ; gTo
                        :grasp "left_gripper" object-name grasp))
                      :result-as-pose-or-transform :pose))) ; bTo * oTg = bTg
      (list (get-object-type-lift-pose object-type arm grasp put-pose)
            (get-object-type-2nd-lift-pose object-type arm grasp put-pose)
            put-pose
            (get-object-type-2nd-pregrasp-pose object-type arm grasp put-pose)
            (get-object-type-pregrasp-pose object-type arm grasp put-pose)))))

#+everything-below-is-commented-out
(
(defgeneric get-object-placement-transform (on-object-type on-object-name
                                            object-type object-name arm grasp))

(defmethod get-object-placement-transform ((on-object-type (eql :chassis-holder))
                                           on-object-name
                                           (object-type (eql :chassis))
                                           object-name
                                           (arm (eql :left))
                                           (grasp (eql :side)))
  (cl-transforms-stamped:make-transform-stamped
   (cram->knowrob on-object-name)
   (cram->knowrob object-name)
   0.0
   (cl-transforms:make-3d-vector 0.0d0 0.0d0 *z-of-chassis-holder*)
   (cl-transforms:matrix->quaternion
    #2A((-1 0 0)
        (0 1 0)
        (0 0 -1)))))

(defmethod get-object-placement-transform ((on-object-type (eql :chassis))
                                           on-object-name
                                           (object-type (eql :axle))
                                           object-name
                                           (arm (eql :left))
                                           (grasp (eql :top)))
  (cl-transforms-stamped:make-transform-stamped
   (cram->knowrob on-object-name)
   (cram->knowrob object-name)
   0.0
   (cl-transforms:make-3d-vector *x-of-chassis-holder-hole* 0 0)
   (cl-transforms:matrix->quaternion
    #2A((-1 0 0)
        (0 1 0)
        (0 0 -1)))))

(defmethod get-object-placement-transform ((on-object-type (eql :chassis))
                                           on-object-name
                                           (object-type (eql :camaro-body))
                                           object-name
                                           (arm (eql :left))
                                           (grasp (eql :top)))
  (cl-transforms-stamped:make-transform-stamped
   (cram->knowrob on-object-name)
   (cram->knowrob object-name)
   0.0
   (cl-transforms:make-3d-vector 0 0 0)
   (cl-transforms:matrix->quaternion
    #2A((0 1 0)
        (-1 0 0)
        (0 0 1)))))
)
