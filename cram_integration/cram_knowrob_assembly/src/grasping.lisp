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

(defparameter *default-z-offset* 0.2 "in meters")
(defparameter *default-small-z-offset* 0.07 "in meters")
(defparameter *default-lift-offsets* `(0.0 0.0 ,*default-z-offset*))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod get-object-type-gripping-effort (object-type) 35)
(defmethod get-object-type-gripping-effort ((object-type (eql :bolt))) 35)
(defmethod get-object-type-gripping-effort ((object-type (eql :upper-body))) 35)
(defmethod get-object-type-gripping-effort ((object-type (eql :bottom-wing))) 35)
(defmethod get-object-type-gripping-effort ((object-type (eql :underbody))) 35)
(defmethod get-object-type-gripping-effort ((object-type (eql :window))) 35)
(defmethod get-object-type-gripping-effort ((object-type (eql :front-wheel))) 35)
(defmethod get-object-type-gripping-effort ((object-type (eql :nut))) 35)
(defmethod get-object-type-gripping-effort ((object-type (eql :chassis))) 35)
(defmethod get-object-type-gripping-effort ((object-type (eql :propeller))) 35)
(defmethod get-object-type-gripping-effort ((object-type (eql :top-wing))) 35)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod get-object-type-gripper-opening (object-type) 0.10)
(defmethod get-object-type-gripper-opening ((object-type (eql :bolt))) 0.02)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-fact-group asm-object-knowledge (obj-int:object-type-grasp)
  ;; (<- (obj-int:object-type-grasp :porsche-body :top))
  ;; (<- (obj-int:object-type-grasp :camaro-body :top))
  ;; (<- (obj-int:object-type-grasp :chassis :side))
  ;; (<- (obj-int:object-type-grasp :axle :top))
  ;; (<- (obj-int:object-type-grasp :wheel :top))
  (<- (obj-int:object-type-grasp :chassis :top))

  )


;; (defmethod get-object-type-to-gripper-transform (object-type object-name arm grasp)
;;   (declare (ignore object-type))
;;   "Default implementation when using KnowRob's get_grasp_position query."
;;   (cram-tf:transform-stamped-inv ; oTg
;;    (get-object-manipulation-transform :grasp  ; gTo
;;                                       (ecase arm
;;                                         (:left "left_gripper")
;;                                         (:right "right_gripper"))
;;                                       object-name
;;                                       grasp)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;; CHASSIS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *chassis-grasp-z-offset* -0.02)

;; TOP grasp
(def-object-type-to-gripper-transforms :chassis '(:left :right) :top
  :grasp-translation `(0.0 0.0 ,*chassis-grasp-z-offset*)
  :grasp-rot-matrix *top-across-x-grasp-rotation*
  :pregrasp-offsets *default-lift-offsets*
  :2nd-pregrasp-offsets *default-lift-offsets*
  :lift-offsets *default-lift-offsets*
  :2nd-lift-offsets *default-lift-offsets*)





#+everything-below-is-commented-out
(
(defmethod get-object-type-gripping-effort ((object-type (eql :porsche-body))) 35)
(defmethod get-object-type-gripping-effort ((object-type (eql :camaro-body))) 35)
(defmethod get-object-type-gripping-effort ((object-type (eql :chassis))) 50)
(defmethod get-object-type-gripping-effort ((object-type (eql :axle))) 50)
(defmethod get-object-type-gripping-effort ((object-type (eql :wheel))) 40)
(defmethod get-object-type-gripping-effort ((object-type (eql :rear-wing))) 35)

(defmethod get-object-type-gripper-opening ((object-type (eql :axle))) 0.02)

(defmethod get-object-type-to-gripper-transform ((object-type (eql :axle))
                                                 object-name
                                                 (arm (eql :left))
                                                 (grasp (eql :top)))
  (cl-transforms-stamped:make-transform-stamped
   object-name
   cram-tf:*robot-left-tool-frame*
   0.0
   (cl-transforms:make-3d-vector 0.0d0 0.0d0 0.0d0)
   (cl-transforms:matrix->quaternion
    #2A((-1 0 0)
        (0 1 0)
        (0 0 -1)))))

(defmethod get-object-type-to-gripper-transform ((object-type (eql :chassis))
                                                 object-name
                                                 (arm (eql :left))
                                                 (grasp (eql :side)))
  (cl-transforms-stamped:make-transform-stamped
   object-name
   cram-tf:*robot-left-tool-frame*
   0.0
   (cl-transforms:make-3d-vector 0.0d0 0.0d0 0.0d0)
   (cl-transforms:matrix->quaternion
    #2A((0 0 1)
        (-1 0 0)
        (0 -1 0)))))

(defmethod get-object-type-to-gripper-transform ((object-type (eql :camaro-body))
                                                 object-name
                                                 (arm (eql :left))
                                                 (grasp (eql :top)))
  (cl-transforms-stamped:make-transform-stamped
   object-name
   cram-tf:*robot-left-tool-frame*
   0.0
   (cl-transforms:make-3d-vector 0.0d0 0.0d0 0.0d0)
   (cl-transforms:matrix->quaternion
    #2A((0 1 0)
        (1 0 0)
        (0 0 -1)))))


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

(defmethod get-object-type-pregrasp-pose ((object-type (eql :short-seat))
                                          (arm (eql :left))
                                          (grasp (eql :top))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose :z-offset *default-z-offset*))
(defmethod get-object-type-lift-pose ((object-type (eql :short-seat))
                                      (arm (eql :left))
                                      (grasp (eql :top))
                                      grasp-pose)
  (cram-tf:translate-pose grasp-pose :z-offset *default-z-offset*))
)
