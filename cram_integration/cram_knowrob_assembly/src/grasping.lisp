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
;; (defmethod get-object-type-gripping-effort ((object-type (eql :bolt))) 35)
;; (defmethod get-object-type-gripping-effort ((object-type (eql :chassis))) 35)
;; (defmethod get-object-type-gripping-effort ((object-type (eql :bottom-wing))) 35)
;; (defmethod get-object-type-gripping-effort ((object-type (eql :underbody))) 35)
;; (defmethod get-object-type-gripping-effort ((object-type (eql :upper-body))) 35)
;; (defmethod get-object-type-gripping-effort ((object-type (eql :top-wing))) 35)
;; (defmethod get-object-type-gripping-effort ((object-type (eql :window))) 35)
;; (defmethod get-object-type-gripping-effort ((object-type (eql :propeller))) 35)
;; (defmethod get-object-type-gripping-effort ((object-type (eql :front-wheel))) 35)
;; (defmethod get-object-type-gripping-effort ((object-type (eql :nut))) 35)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod get-object-type-gripper-opening (object-type) 0.10)
(defmethod get-object-type-gripper-opening ((object-type (eql :bolt))) 0.02)
;; (defmethod get-object-type-gripper-opening ((object-type (eql :chassis))) 0.1)
;; (defmethod get-object-type-gripper-opening ((object-type (eql :bottom-wing))) 0.1)
;; (defmethod get-object-type-gripper-opening ((object-type (eql :underbody))) 0.1)
;; (defmethod get-object-type-gripper-opening ((object-type (eql :upper-body))) 0.1)
;; (defmethod get-object-type-gripper-opening ((object-type (eql :top-wing))) 0.1)
;; (defmethod get-object-type-gripper-opening ((object-type (eql :window))) 0.1)
;; (defmethod get-object-type-gripper-opening ((object-type (eql :propeller))) 0.1)
;; (defmethod get-object-type-gripper-opening ((object-type (eql :front-wheel))) 0.1)
;; (defmethod get-object-type-gripper-opening ((object-type (eql :nut))) 0.1)

;;;;;;;;;;;;;;;;;;;;;;;;;;;; CHASSIS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *chassis-grasp-z-offset* -0.02)

;; TOP grasp
(def-object-type-to-gripper-transforms :chassis '(:left :right) :top
  :grasp-translation `(0.0 0.0 ,*chassis-grasp-z-offset*)
  :grasp-rot-matrix *z-across-x-grasp-rotation*
  :pregrasp-offsets *default-lift-offsets*
  :2nd-pregrasp-offsets *default-lift-offsets*
  :lift-offsets *default-lift-offsets*
  :2nd-lift-offsets *default-lift-offsets*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;; BOTTOM-WING ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *bottom-wing-grasp-x-offset* 0.07)
(defparameter *bottom-wing-grasp-y-offset* 0.01)
(defparameter *bottom-wing-grasp-z-offset* 0.02)

;; SIDE grasp
(def-object-type-to-gripper-transforms :bottom-wing :left :right-side
  :grasp-translation `(,(- *bottom-wing-grasp-x-offset*)
                       ,*bottom-wing-grasp-y-offset*
                       ,*bottom-wing-grasp-z-offset*)
  :grasp-rot-matrix *y-across-x-grasp-rotation*
  :pregrasp-offsets `(0 ,*default-z-offset* ,*default-z-offset*)
  :2nd-pregrasp-offsets `(0 ,*default-z-offset* 0.0)
  :lift-offsets *default-lift-offsets*
  :2nd-lift-offsets *default-lift-offsets*)

(def-object-type-to-gripper-transforms :bottom-wing :right :right-side
  :grasp-translation `(,*bottom-wing-grasp-x-offset*
                       ,(- *bottom-wing-grasp-y-offset*)
                       ,*bottom-wing-grasp-z-offset*)
  :grasp-rot-matrix *-y-across-x-grasp-rotation*
  :pregrasp-offsets `(0 ,(- *default-z-offset*) ,*default-z-offset*)
  :2nd-pregrasp-offsets `(0 ,(- *default-z-offset*) 0.0)
  :lift-offsets *default-lift-offsets*
  :2nd-lift-offsets *default-lift-offsets*)

;; BACK grasp
(def-object-type-to-gripper-transforms :bottom-wing '(:left :right) :back
  :grasp-translation `(,(- *bottom-wing-grasp-x-offset*)
                       0.0
                       ,*bottom-wing-grasp-z-offset*)
  :grasp-rot-matrix *-x-across-y-grasp-rotation*
  :pregrasp-offsets `(,(- *default-z-offset*) 0.0 ,*default-z-offset*)
  :2nd-pregrasp-offsets `(,(- *default-z-offset*) 0.0 0.0)
  :lift-offsets *default-lift-offsets*
  :2nd-lift-offsets *default-lift-offsets*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;; UNDERBODY ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *underbody-grasp-y-offset* 0.03)
(defparameter *underbody-grasp-z-offset* 0.0)

;; TOP grasp
(def-object-type-to-gripper-transforms :underbody :left :top
  :grasp-translation `(0.0 ,*underbody-grasp-y-offset* ,*underbody-grasp-z-offset*)
  :grasp-rot-matrix *z-across-x-grasp-rotation*
  :pregrasp-offsets *default-lift-offsets*
  :2nd-pregrasp-offsets *default-lift-offsets*
  :lift-offsets *default-lift-offsets*
  :2nd-lift-offsets *default-lift-offsets*)

(def-object-type-to-gripper-transforms :underbody :right :top
  :grasp-translation `(0.0 ,(- *underbody-grasp-y-offset*) ,*underbody-grasp-z-offset*)
  :grasp-rot-matrix *z-across-x-grasp-rotation*
  :pregrasp-offsets *default-lift-offsets*
  :2nd-pregrasp-offsets *default-lift-offsets*
  :lift-offsets *default-lift-offsets*
  :2nd-lift-offsets *default-lift-offsets*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;; UPPER-BODY ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *upper-body-grasp-x-offset* 0.09)
(defparameter *upper-body-grasp-z-offset* 0.0)

;; TOP grasp
(def-object-type-to-gripper-transforms :upper-body '(:left :right) :top
  :grasp-translation `(,(- *upper-body-grasp-x-offset*) 0.0 ,*upper-body-grasp-z-offset*)
  :grasp-rot-matrix *z-across-x-grasp-rotation*
  :pregrasp-offsets *default-lift-offsets*
  :2nd-pregrasp-offsets *default-lift-offsets*
  :lift-offsets *default-lift-offsets*
  :2nd-lift-offsets *default-lift-offsets*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;; TOP-WING ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *top-wing-grasp-x-offset* 0.08)
(defparameter *top-wing-grasp-y-offset* 0.01)
(defparameter *top-wing-grasp-z-offset* 0.03)

;; BACK grasp
(def-object-type-to-gripper-transforms :top-wing '(:left :right) :back
  :grasp-translation `(,(- *top-wing-grasp-x-offset*)
                       0.0
                       ,*top-wing-grasp-z-offset*)
  :grasp-rot-matrix *-x-across-y-grasp-rotation*
  :pregrasp-offsets `(,(- *default-z-offset*) 0.0 ,*default-z-offset*)
  :2nd-pregrasp-offsets `(,(- *default-z-offset*) 0.0 0.0)
  :lift-offsets *default-lift-offsets*
  :2nd-lift-offsets *default-lift-offsets*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;; WINDOW ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *window-grasp-x-offset* 0.025)
(defparameter *window-grasp-y-offset* 0.012)
(defparameter *window-grasp-z-offset* 0.015)

(def-object-type-to-gripper-transforms :window '(:left :right) :top
  :grasp-translation `(,(- *window-grasp-x-offset*)
                        ,*window-grasp-y-offset*
                        ,(- *window-grasp-z-offset*))
  :grasp-rot-matrix *z-diagonal-grasp-rotation*
  :pregrasp-offsets *default-lift-offsets*
  :2nd-pregrasp-offsets *default-lift-offsets*
  :lift-offsets *default-lift-offsets*
  :2nd-lift-offsets *default-lift-offsets*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;; BOLT ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-object-type-to-gripper-transforms :bolt '(:left :right) :top
  :grasp-translation `(0.0 0.0 0.0)
  :grasp-rot-matrix *z-across-x-grasp-rotation*
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
