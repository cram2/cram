;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;               2019, Thomas Lipps <tlipps@uni-bremen.de>
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

(defparameter *default-z-offset* 0.1 "in meters")
(defparameter *default-small-z-offset* 0.07 "in meters")
(defparameter *default-lift-offsets* `(0.0 0.0 ,*default-z-offset*))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-fact-group assembly-object-type-hierarchy (man-int:object-type-direct-subtype)
  (<- (man-int:object-type-direct-subtype :assembly-item :bolt))
  (<- (man-int:object-type-direct-subtype :assembly-item :chassis))
  (<- (man-int:object-type-direct-subtype :assembly-item :bottom-wing))
  (<- (man-int:object-type-direct-subtype :assembly-item :underbody))
  (<- (man-int:object-type-direct-subtype :assembly-item :upper-body))
  (<- (man-int:object-type-direct-subtype :assembly-item :top-wing))
  (<- (man-int:object-type-direct-subtype :assembly-item :window))
  (<- (man-int:object-type-direct-subtype :assembly-item :propeller))
  (<- (man-int:object-type-direct-subtype :assembly-item :front-wheel))
  (<- (man-int:object-type-direct-subtype :assembly-item :nut)))

(def-fact-group attachmend-knowledge (man-int:unidirectional-attachment)

  (<- (man-int:unidirectional-attachment ?attachment-type)
    (member ?attachment-type (:horizontal-attachment :vertical-attachment))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-action-gripping-effort :heuristics 20 ((object-type (eql :assembly-item)))
  35)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-action-gripper-opening :heuristics 20 ((object-type (eql :assembly-item)))
  0.1)
(defmethod man-int:get-action-gripper-opening :heuristics 20 ((object-type (eql :bolt)))
  0.02)
(defmethod man-int:get-action-gripper-opening :heuristics 20 ((object-type (eql :window)))
  0.02)

;;;;;;;;;;;;;;;;;;;;;;;;;;;; CHASSIS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *chassis-grasp-z-offset* -0.02)

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms :chassis '(:left :right) :top
  :grasp-translation `(0.0 0.0 ,*chassis-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets *default-lift-offsets*
  :2nd-pregrasp-offsets *default-lift-offsets*
  :lift-translation *default-lift-offsets*
  :2nd-lift-translation *default-lift-offsets*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;; BOTTOM-WING ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *bottom-wing-grasp-x-offset* 0.07)
(defparameter *bottom-wing-grasp-y-offset* 0.01)
(defparameter *bottom-wing-grasp-z-offset* 0.02)

;; SIDE grasp
(man-int:def-object-type-to-gripper-transforms :bottom-wing :left :right-side
  :grasp-translation `(,(- *bottom-wing-grasp-x-offset*)
                       ,*bottom-wing-grasp-y-offset*
                       ,*bottom-wing-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-x-grasp-rotation*
  :pregrasp-offsets `(0 ,*default-z-offset* ,*default-z-offset*)
  :2nd-pregrasp-offsets `(0 ,*default-z-offset* 0.0)
  :lift-translation *default-lift-offsets*
  :2nd-lift-translation *default-lift-offsets*)

(man-int:def-object-type-to-gripper-transforms :bottom-wing :right :right-side
  :grasp-translation `(,*bottom-wing-grasp-x-offset*
                       ,(- *bottom-wing-grasp-y-offset*)
                       ,*bottom-wing-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-x-grasp-rotation*
  :pregrasp-offsets `(0 ,(- *default-z-offset*) ,*default-z-offset*)
  :2nd-pregrasp-offsets `(0 ,(- *default-z-offset*) 0.0)
  :lift-translation *default-lift-offsets*
  :2nd-lift-translation *default-lift-offsets*)

;; BACK grasp
(man-int:def-object-type-to-gripper-transforms :bottom-wing '(:left :right) :back
  :grasp-translation `(,(- *bottom-wing-grasp-x-offset*)
                       0.0
                       ,*bottom-wing-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-y-grasp-rotation*
  :pregrasp-offsets `(,(- *default-z-offset*) 0.0 ,*default-z-offset*)
  :2nd-pregrasp-offsets `(,(- *default-z-offset*) 0.0 0.0)
  :lift-translation *default-lift-offsets*
  :2nd-lift-translation *default-lift-offsets*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;; UNDERBODY ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *underbody-grasp-y-offset* 0.03)
(defparameter *underbody-grasp-z-offset* 0.0)

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms :underbody :left :top
  :grasp-translation `(0.0 ,*underbody-grasp-y-offset* ,*underbody-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets *default-lift-offsets*
  :2nd-pregrasp-offsets *default-lift-offsets*
  :lift-translation *default-lift-offsets*
  :2nd-lift-translation *default-lift-offsets*)

(man-int:def-object-type-to-gripper-transforms :underbody :right :top
  :grasp-translation `(0.0 ,(- *underbody-grasp-y-offset*) ,*underbody-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets *default-lift-offsets*
  :2nd-pregrasp-offsets *default-lift-offsets*
  :lift-translation *default-lift-offsets*
  :2nd-lift-translation *default-lift-offsets*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;; UPPER-BODY ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *upper-body-grasp-x-offset* 0.09)
(defparameter *upper-body-grasp-z-offset* 0.0)

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms :upper-body '(:left :right) :top
  :grasp-translation `(,(- *upper-body-grasp-x-offset*) 0.0 ,*upper-body-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets *default-lift-offsets*
  :2nd-pregrasp-offsets *default-lift-offsets*
  :lift-translation *default-lift-offsets*
  :2nd-lift-translation *default-lift-offsets*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;; TOP-WING ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *top-wing-grasp-x-offset* 0.08)
(defparameter *top-wing-grasp-y-offset* 0.01)
(defparameter *top-wing-grasp-z-offset* 0.03)

;; BACK grasp
(man-int:def-object-type-to-gripper-transforms :top-wing '(:left :right) :back
  :grasp-translation `(,(- *top-wing-grasp-x-offset*)
                       0.0
                       ,*top-wing-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-y-grasp-rotation*
  :pregrasp-offsets `(,(- *default-z-offset*) 0.0 ,*default-z-offset*)
  :2nd-pregrasp-offsets `(,(- *default-z-offset*) 0.0 0.0)
  :lift-translation *default-lift-offsets*
  :2nd-lift-translation *default-lift-offsets*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;; WINDOW ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *window-grasp-x-offset* 0.017)
(defparameter *window-grasp-y-offset* 0.005)
(defparameter *window-grasp-z-offset* 0.015)

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms :window '(:left :right) :top
  :grasp-translation `(,(- *window-grasp-x-offset*)
                        ,*window-grasp-y-offset*
                        ,(- *window-grasp-z-offset*))
  :grasp-rot-matrix man-int:*z-diagonal-grasp-rotation*
  :pregrasp-offsets *default-lift-offsets*
  :2nd-pregrasp-offsets *default-lift-offsets*
  :lift-translation *default-lift-offsets*
  :2nd-lift-translation *default-lift-offsets*)

;;;;;;;;;;;;;;;;;;;;;;;;;;; FRONT-WHEEL ;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *front-wheel-grasp-x-offset* 0.01)
(defparameter *front-wheel-grasp-y-offset* 0.01)
(defparameter *front-wheel-grasp-z-offset* 0.004)

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms :front-wheel '(:left :right) :top
  :grasp-translation `(,(- *front-wheel-grasp-x-offset*)
                       ,*front-wheel-grasp-y-offset*
                       ,(- *front-wheel-grasp-z-offset*))
  :grasp-rot-matrix man-int:*z-diagonal-grasp-rotation*
  :pregrasp-offsets *default-lift-offsets*
  :2nd-pregrasp-offsets *default-lift-offsets*
  :lift-translation *default-lift-offsets*
  :2nd-lift-translation *default-lift-offsets*)

;;;;;;;;;;;;;;;;;;;;;;;;;; PROPELLER ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *propeller-grasp-x-offset* 0.013)
(defparameter *propeller-grasp-y-offset* 0.0)
(defparameter *propeller-grasp-z-offset* 0.003)

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms :propeller '(:left :right) :top
  :grasp-translation `(,*propeller-grasp-x-offset*
                       ,*propeller-grasp-y-offset*
                       ,(- *propeller-grasp-z-offset*))
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets *default-lift-offsets*
  :2nd-pregrasp-offsets *default-lift-offsets*
  :lift-translation *default-lift-offsets*
  :2nd-lift-translation *default-lift-offsets*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;; BOLT ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms :bolt '(:left :right) :top
  :grasp-translation `(0.0 0.0 0.003)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets *default-lift-offsets*
  :2nd-pregrasp-offsets *default-lift-offsets*
  :lift-translation *default-lift-offsets*
  :2nd-lift-translation *default-lift-offsets*)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(man-int:def-object-type-in-other-object-transform :chassis :holder-plane-horizontal :horizontal-attachment
  :attachment-translation `(0.084 0.0 0.022)
  :attachment-rot-matrix man-int:*rotation-around-z-90-matrix*)

(man-int:def-object-type-in-other-object-transform :bottom-wing :chassis :wing-attachment
  :attachment-translation `(0.0 -0.02 0.04;; 0.0
                                )
  :attachment-rot-matrix man-int:*identity-matrix*)

(defmethod man-int::get-z-offset-for-placing-distance :heuristics 20
  ((other-object (eql :chassis))
   (object (eql :bottom-wing))
   (attachment (eql :wing-attachment)))
  0.02)

(man-int:def-object-type-in-other-object-transform :underbody :bottom-wing :body-attachment
  :attachment-translation `(0.0 -0.025 0.02)
  :attachment-rot-matrix man-int:*rotation-around-z+90-matrix*)

(man-int:def-object-type-in-other-object-transform :upper-body :underbody :body-on-body
  :attachment-translation `(-0.025 0.0 0.0425)
  :attachment-rot-matrix man-int:*identity-matrix*)

(man-int:def-object-type-in-other-object-transform :propeller :motor-grill :propeller-attachment
  :attachment-translation `(0.0 0.0 0.002)
  :attachment-rot-matrix man-int:*identity-matrix*)

(man-int:def-object-type-in-other-object-transform :front-wheel :chassis :left-wheel-attachment
  :attachment-translation `(-0.0 -0.15 0.00)
  :attachment-rot-matrix man-int:*rotation-around-x+90-matrix*)

(man-int:def-object-type-in-other-object-transform :front-wheel :chassis :right-wheel-attachment
  :attachment-translation `(-0.0 -0.15 0.00)
  :attachment-rot-matrix  man-int:*rotation-around-x+90-matrix*)

(man-int:def-object-type-in-other-object-transform :top-wing :holder-plane-vertical :vertical-attachment
  :attachment-translation `(0.025 0 0.183)
  :attachment-rot-matrix man-int:*rotation-around-z-180-and-x+90-matrix*)

(man-int:def-object-type-in-other-object-transform :bolt :upper-body :rear-thread
  :attachment-translation `(-0.0525 0.0 -0.01;; -0.025
                                    )
  :attachment-rot-matrix man-int:*identity-matrix*)

(man-int:def-object-type-in-other-object-transform :top-wing :upper-body :wing-attachment
  :attachment-translation `(0.05 0.0 0.0025)
  :attachment-rot-matrix man-int:*rotation-around-z-90-matrix*)

(man-int:def-object-type-in-other-object-transform :top-wing :holder-plane-horizontal :horizontal-attachment
  :attachment-translation `(0.035 0.0 0.128)
  :attachment-rot-matrix man-int:*rotation-around-z-90-matrix*)

(man-int:def-object-type-in-other-object-transform :bolt :top-wing :middle-thread
  :attachment-translation `(0.0 0.025 0.01;; -0.005
                                )
  :attachment-rot-matrix man-int:*identity-matrix*)

(man-int:def-object-type-in-other-object-transform :window :top-wing :window-attachment
  :attachment-translation `(0.0 -0.0525 0.0075)
  :attachment-rot-matrix man-int:*rotation-around-z+90-matrix*)

(man-int:def-object-type-in-other-object-transform :bolt :window :window-thread
  :attachment-translation `(-0.0125 0.0 -0.005;; -0.02
                                    )
  :attachment-rot-matrix man-int:*identity-matrix*)

(man-int:def-object-type-in-other-object-transform :bolt :propeller :propeller-thread
  :attachment-translation `(0.0 0.0 0.01;; -0.02
                                    )
  :attachment-rot-matrix man-int:*identity-matrix*)

