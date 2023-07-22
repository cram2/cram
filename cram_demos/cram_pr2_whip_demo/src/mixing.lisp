;;;
;;; Copyright (c) 2023, Tina Van <van@uni-bremen.de>
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

(in-package :cram-manipulation-interfaces)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
         ;approach pose in bto

(defmethod get-object-type-robot-frame-mix-approach-transform
    ((object-type (eql :big-bowl)))
  '((0 0 0)(1 0 0 0)));0.02 -0.12 0.161)(1 0 0 0)))

(defmethod get-object-type-robot-frame-mix-approach-transform
    ((object-type (eql :saucepan)))
  '((0 0 0)(1 0 0 0)))

(defmethod get-object-type-robot-frame-mix-approach-transform
    ((object-type (eql :bowl-round)))
  '((0 0 0)(1 0 0 0)))

(defmethod get-object-type-robot-frame-mix-approach-transform
    ((object-type (eql :wine-glas)))
  '((0 0 0.05)(1 0 0 0)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;in object coordinates mix information for lower circle

(defmethod get-object-type-robot-frame-mix-rim-bottom-transform
    ((object-type (eql :big-bowl)))
  '((0.063 0.06 -0.045)(1 0 0 0))) ;16 cm diameter 6 cm below origin

(defmethod get-object-type-robot-frame-mix-rim-bottom-transform
    ((object-type (eql :saucepan)))
  '((0.05 0.08 -0.035)(1 0 0 0)))

(defmethod get-object-type-robot-frame-mix-rim-bottom-transform
    ((object-type (eql :bowl-round)))
  '((0.04 0.04 -0.02)(1 0 0 0)))

(defmethod get-object-type-robot-frame-mix-rim-bottom-transform
    ((object-type (eql :wine-glas)))
  '((0.1 0.023 0.02)(1 0 0 0)))

;decided to use the z axis as radius measure
;(important for mix center point calculation)
;(topheight, radius, bottomheight )(rotation) - think of the thickness

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; utensils
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;-bowl bottom can be looked up in rim-bottom-transform 
;(lenght from origin to tip of the tool, width, depth)(rotation)

(defmethod get-object-type-robot-frame-mix-tool-transform
    ((object-type (eql :whisk)))
  '((0.15 0.03 0.03)(1 0 0 0)))

(defmethod get-object-type-robot-frame-mix-tool-transform
    ((object-type (eql :fork)))
  '((0.12 0.02 0.015)(1 0 0 0)))

(defmethod get-object-type-robot-frame-mix-tool-transform
    ((object-type (eql :ladle)))
  '((0.13 0.044 0.044)(1 0 0 0)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; big-bowl ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *big-bowl-grasp-xy-offset* 0.12 "in meters")
(defparameter *big-bowl-pregrasp-z-offset* 0.20 "in meters")
(defparameter *big-bowl-grasp-z-offset* 0.05);0.12 "in meters") ;height
(defparameter *big-bowl-grasp-thickness* 0.01 "in meter")
;;lifing is zero movment

                                        ;side-grasps

(man-int:def-object-type-to-gripper-transforms '(:big-bowl)
    '(:left :right) :right-top
  :grasp-translation `(0.0,*big-bowl-grasp-xy-offset* ,*big-bowl-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*big-bowl-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*big-bowl-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 ,0.0)
  :2nd-lift-translation `(0.0 0.0 ,0.0))

;; right-TOP grasp
(man-int:def-object-type-to-gripper-transforms '(:big-bowl)
    '(:left :right) :left-top
  :grasp-translation `(0,-0.12, *big-bowl-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*big-bowl-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 , *big-bowl-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 0.0)
  :2nd-lift-translation `(0.0 ,0.0, 0.0))

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; whisk ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *whisk-grasp-y-offset* 0.015 ;; -0.015
  "in meters") ; because TCP is not at the edge
(defparameter *whisk-pregrasp-z-offset* 0.20 "in meters")

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms '(:whisk)
    '(:left :right) :top
  :location-type :counter-top
  :grasp-translation  `(-0.05 0 0) ;-0.05 0 -0.02)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets `(-0.05 0, *whisk-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(-0.05 0 ,*whisk-pregrasp-z-offset*)
  :lift-translation `(-0.05 0, *whisk-pregrasp-z-offset*)
  :2nd-lift-translation `(-0.05, *whisk-grasp-y-offset* , *whisk-pregrasp-z-offset*))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;ladle;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *ladle-grasp-y-offset* 0.015 ;; -0.015
  "in meters") ; because TCP is not at the edge
(defparameter *ladle-pregrasp-z-offset* 0.20 "in meters")

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms '(:ladle)
    '(:left :right) :top
  :location-type :counter-top
  :grasp-translation  `(-0.05 0 -0.04)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets `(-0.05 0, *ladle-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(-0.05 0, *ladle-pregrasp-z-offset*)
  :lift-translation `(-0.05 0, *ladle-pregrasp-z-offset*)
  :2nd-lift-translation `(-0.05, *ladle-grasp-y-offset*, *ladle-pregrasp-z-offset*))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;saucepan;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defparameter *saucepan-grasp-x-offset* -0.2 "in meters")
(defparameter *saucepan-pregrasp-z-offset* 0.30 "in meters")
(defparameter *saucepan-grasp-z-offset* 0.022 "in meters")

                                        ;side-grasps
(man-int:def-object-type-to-gripper-transforms '(:saucepan)
    '(:left :right) :handle-left
  :grasp-translation `(-0.2,0.0 ,*saucepan-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*saucepan-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0, 0.0 ,*saucepan-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 ,0.0)
  :2nd-lift-translation `(0.0 0.0 ,0.0))

(man-int:def-object-type-to-gripper-transforms '(:saucepan)
    '(:left :right) :handle-right
  :grasp-translation `(-0.2,0.0 ,*saucepan-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*saucepan-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0, 0.0 ,*saucepan-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 ,0.0)
  :2nd-lift-translation `(0.0 0.0 ,0.0))

(man-int:def-object-type-to-gripper-transforms '(:saucepan)
    '(:left :right) :handle-top
  :grasp-translation `(-0.2,0.0 ,*saucepan-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*saucepan-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0, 0.0 ,*saucepan-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 ,0.0)
  :2nd-lift-translation `(0.0 0.0 ,0.0))

(man-int:def-object-type-to-gripper-transforms '(:saucepan)
    '(:left :right) :handle-bottom
  :grasp-translation `(-0.2,0.0 ,*saucepan-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*saucepan-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0, 0.0 ,*saucepan-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 ,0.0)
  :2nd-lift-translation `(0.0 0.0 ,0.0))

;;;;;;;;;;;;;;;;;;;;;;;;;;;; wine-glas ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *wine-glas-pregrasp-x-offset* 0.01 "in meters") ;counts for the stem
(defparameter *wine-glas-grasp-z-offset* -0.05 "in meters") ;counts for the stem
(defparameter *wine-glas-lift-z-offset* 0.03 "in meters")
                                        ;thickness of glas is 1 cm ish

;; (man-int:def-object-type-to-gripper-transforms :wine-glas
;;     '(:left :right) :stem-front
;;   :grasp-translation `(0.0 0.0 ,*wine-glas-grasp-z-offset*)
;;   :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
;;   :pregrasp-offsets `(0.0 0.0 ,*wine-glas-lift-z-offset*)
;;   :2nd-pregrasp-offsets `(0.0 0.0 ,*wine-glas-lift-z-offset*)
;;   :lift-translation `(0.0 0.0 ,0.0)
;;   :2nd-lift-translation `( 0.01 0.0 ,0.0))
;;TOP grasp
(man-int:def-object-type-to-gripper-transforms :wine-glas
    '(:left :right) :stem-left
  :grasp-translation `(0.0 0.0 ,*wine-glas-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-z-grasp-rotation*
  :pregrasp-offsets `(-0.1 0.0 ,*wine-glas-lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*wine-glas-lift-z-offset*)
  :lift-translation `(0.0 0.0 ,0.0)
  :2nd-lift-translation `( 0.01 0.0 ,0.0))

;; (man-int:def-object-type-to-gripper-transforms :wine-glas
;;     '(:left :right) :stem-right
;;   :grasp-translation `(0.0 0.0 ,*wine-glas-grasp-z-offset*)
;;   :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
;;   :pregrasp-offsets `(0.1 0.0 ,*wine-glas-lift-z-offset*)
;;   :2nd-pregrasp-offsets `(0.0 0.0 ,*wine-glas-lift-z-offset*)
;;   :lift-translation `(0.0 0.0 ,0.0)
;;   :2nd-lift-translation `( 0.01 0.0 ,0.0))


