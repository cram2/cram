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

(defparameter *default-retail-z-offset* 0.05 "in meters")
(defparameter *default-retail-lift-offsets* `(0.0 0.0 ,*default-retail-z-offset*))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-fact-group retail-object-type-hierarchy (man-int:object-type-direct-subtype)
  (<- (man-int:object-type-direct-subtype :retail-item :dish-washer-tabs))
  (<- (man-int:object-type-direct-subtype :retail-item :balea-bottle))
  (<- (man-int:object-type-direct-subtype :retail-item :deodorant))
  (<- (man-int:object-type-direct-subtype :retail-item :juice-box)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-action-gripper-opening :heuristics 20 ((object-type (eql :retail-item)))
  0.1)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-action-gripping-effort :heuristics 20 ((object-type (eql :retail-item)))
  ;; it's actually not the effort but the slippage parameter,
  ;; because that's what donbot's gripper speaks...
  0.72)

(defmethod man-int:get-action-gripping-effort :heuristics 20 ((object-type (eql :dish-washer-tabs)))
  0.72)
(defmethod man-int:get-action-gripping-effort :heuristics 20 ((object-type (eql :balea-bottle)))
  0.72)
(defmethod man-int:get-action-gripping-effort :heuristics 20 ((object-type (eql :deodorant)))
  0.9)
(defmethod man-int:get-action-gripping-effort :heuristics 20 ((object-type (eql :juice-box)))
  0.82)

;; ;;;;;;;;;;;;;;;;;;;;;;;;;;;; DISH-WASHER-TABS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *dish-washer-tabs-grasp-x-offset* 0.01 "in meters")
(defparameter *dish-washer-tabs-grasp-z-offset* 0.06 "in meters")
(defparameter *dish-washer-tabs-pregrasp-x-offset* 0.1 "in meters")

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms :dish-washer-tabs '(:left :right) :top
  :grasp-translation `(0.0 0.0 ,*dish-washer-tabs-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets *default-retail-lift-offsets*
  :2nd-pregrasp-offsets *default-retail-lift-offsets*
  :lift-offsets *default-retail-lift-offsets*
  :2nd-lift-offsets *default-retail-lift-offsets*)

;; BACK grasp
(man-int:def-object-type-to-gripper-transforms :dish-washer-tabs '(:left :right) :back
  :grasp-translation `(,(- *dish-washer-tabs-grasp-x-offset*)
                       0.0
                       ,*dish-washer-tabs-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation-2*
  :pregrasp-offsets `(,(- *dish-washer-tabs-pregrasp-x-offset*)
                       0.0
                       ,*default-retail-z-offset*)
  :2nd-pregrasp-offsets `(,(- *dish-washer-tabs-pregrasp-x-offset*)
                           0.0
                           ,*dish-washer-tabs-grasp-z-offset*)
  :lift-offsets *default-retail-lift-offsets*
  :2nd-lift-offsets *default-retail-lift-offsets*)

;; FRONT grasp
(man-int:def-object-type-to-gripper-transforms :dish-washer-tabs '(:left :right) :front
  :grasp-translation `(,*dish-washer-tabs-grasp-x-offset*
                       0.0
                       ,*dish-washer-tabs-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation-2*
  :pregrasp-offsets `(,*dish-washer-tabs-pregrasp-x-offset*
                      0.0
                      ,*default-retail-z-offset*)
  :2nd-pregrasp-offsets `(,*dish-washer-tabs-pregrasp-x-offset*
                          0.0
                          ,*dish-washer-tabs-grasp-z-offset*)
  :lift-offsets *default-retail-lift-offsets*
  :2nd-lift-offsets *default-retail-lift-offsets*)



;; ;;;;;;;;;;;;;;;;;;;;;;;;;;;; BALEA-BOTTLE ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *balea-bottle-grasp-z-offset* 0.08 "in meters")
(defparameter *balea-bottle-pregrasp-x-offset* 0.1 "in meters")

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms :balea-bottle '(:left :right) :top
  :grasp-translation `(0.0 0.0 ,*balea-bottle-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets *default-retail-lift-offsets*
  :2nd-pregrasp-offsets *default-retail-lift-offsets*
  :lift-offsets *default-retail-lift-offsets*
  :2nd-lift-offsets *default-retail-lift-offsets*)

;; BACK grasp
(man-int:def-object-type-to-gripper-transforms :balea-bottle '(:left :right) :back
  :grasp-translation `(0.0 0.0 ,*dish-washer-tabs-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation-2*
  :pregrasp-offsets `(,(- *dish-washer-tabs-pregrasp-x-offset*)
                       0.0
                       ,*default-retail-z-offset*)
  :2nd-pregrasp-offsets `(,(- *dish-washer-tabs-pregrasp-x-offset*)
                           0.0
                           ,*dish-washer-tabs-grasp-z-offset*)
  :lift-offsets *default-retail-lift-offsets*
  :2nd-lift-offsets *default-retail-lift-offsets*)

;; FRONT grasp
(man-int:def-object-type-to-gripper-transforms :balea-bottle '(:left :right) :front
  :grasp-translation `(0.0
                       0.0
                       ,*dish-washer-tabs-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation-2*
  :pregrasp-offsets `(,*dish-washer-tabs-pregrasp-x-offset*
                      0.0
                      ,*default-retail-z-offset*)
  :2nd-pregrasp-offsets `(,*dish-washer-tabs-pregrasp-x-offset*
                          0.0
                          ,*dish-washer-tabs-grasp-z-offset*)
  :lift-offsets *default-retail-lift-offsets*
  :2nd-lift-offsets *default-retail-lift-offsets*)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(man-int:def-object-type-in-other-object-transform :dish-washer-tabs :robot
  :donbot-tray-left
  :attachment-translation `(0.02 0.05 0.086)
  :attachment-rot-matrix '((0.43809 0.89892 0.005072)
                           (-0.89879 0.43811461 -0.01522)
                           (-0.0159 0.0021 0.999871)))

(man-int:def-object-type-in-other-object-transform :balea-bottle :robot
  :donbot-tray-left
  :attachment-translation `(0.1 0.1 0.1)
  :attachment-rot-matrix '((0.43809 0.89892 0.005072)
                           (-0.89879 0.43811461 -0.01522)
                           (-0.0159 0.0021 0.999871)))
