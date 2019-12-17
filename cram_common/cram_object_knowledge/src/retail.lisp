;;;
;;; Copyright (c) 2019, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

;; (defparameter *default-z-offset* 0.1 "in meters")
;; (defparameter *default-small-z-offset* 0.07 "in meters")
;; (defparameter *default-lift-offsets* `(0.0 0.0 ,*default-z-offset*))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-fact-group assembly-object-type-hierarchy (man-int:object-type-direct-subtype)
  (<- (man-int:object-type-direct-subtype :retail-item :deo)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-action-gripping-effort :heuristics 20 ((object-type (eql :retail-item)))
  ;; it's actually not the effort but the slippage parameter,
  ;; because that's what donbot's gripper speaks...
  0.72)

(defmethod man-int:get-action-gripping-effort :heuristics 20 ((object-type (eql :denkmit-geschirr)))
  0.72)
(defmethod man-int:get-action-gripping-effort :heuristics 20 ((object-type (eql :gesichtswasser)))
  0.72)
(defmethod man-int:get-action-gripping-effort :heuristics 20 ((object-type (eql :deo)))
  0.9)
(defmethod man-int:get-action-gripping-effort :heuristics 20 ((object-type (eql :saft)))
  0.82)

;; ;;;;;;;;;;;;;;;;;;;;;;;;;;;; CHASSIS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; (defparameter *chassis-grasp-z-offset* -0.02)

;; ;; TOP grasp
;; (man-int:def-object-type-to-gripper-transforms :chassis '(:left :right) :top
;;   :grasp-translation `(0.0 0.0 ,*chassis-grasp-z-offset*)
;;   :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
;;   :pregrasp-offsets *default-lift-offsets*
;;   :2nd-pregrasp-offsets *default-lift-offsets*
;;   :lift-offsets *default-lift-offsets*
;;   :2nd-lift-offsets *default-lift-offsets*)

;; ;;;;;;;;;;;;;;;;;;;;;;;;;;;; BOTTOM-WING ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; (defparameter *bottom-wing-grasp-x-offset* 0.07)
;; (defparameter *bottom-wing-grasp-y-offset* 0.01)
;; (defparameter *bottom-wing-grasp-z-offset* 0.02)

;; ;; SIDE grasp
;; (man-int:def-object-type-to-gripper-transforms :bottom-wing :left :right-side
;;   :grasp-translation `(,(- *bottom-wing-grasp-x-offset*)
;;                        ,*bottom-wing-grasp-y-offset*
;;                        ,*bottom-wing-grasp-z-offset*)
;;   :grasp-rot-matrix man-int:*y-across-x-grasp-rotation*
;;   :pregrasp-offsets `(0 ,*default-z-offset* ,*default-z-offset*)
;;   :2nd-pregrasp-offsets `(0 ,*default-z-offset* 0.0)
;;   :lift-offsets *default-lift-offsets*
;;   :2nd-lift-offsets *default-lift-offsets*)

;; (man-int:def-object-type-to-gripper-transforms :bottom-wing :right :right-side
;;   :grasp-translation `(,*bottom-wing-grasp-x-offset*
;;                        ,(- *bottom-wing-grasp-y-offset*)
;;                        ,*bottom-wing-grasp-z-offset*)
;;   :grasp-rot-matrix man-int:*-y-across-x-grasp-rotation*
;;   :pregrasp-offsets `(0 ,(- *default-z-offset*) ,*default-z-offset*)
;;   :2nd-pregrasp-offsets `(0 ,(- *default-z-offset*) 0.0)
;;   :lift-offsets *default-lift-offsets*
;;   :2nd-lift-offsets *default-lift-offsets*)

;; ;; BACK grasp
;; (man-int:def-object-type-to-gripper-transforms :bottom-wing '(:left :right) :back
;;   :grasp-translation `(,(- *bottom-wing-grasp-x-offset*)
;;                        0.0
;;                        ,*bottom-wing-grasp-z-offset*)
;;   :grasp-rot-matrix man-int:*-x-across-y-grasp-rotation*
;;   :pregrasp-offsets `(,(- *default-z-offset*) 0.0 ,*default-z-offset*)
;;   :2nd-pregrasp-offsets `(,(- *default-z-offset*) 0.0 0.0)
;;   :lift-offsets *default-lift-offsets*
;;   :2nd-lift-offsets *default-lift-offsets*)
