;;;
;;; Copyright (c) 2019, Vanessa Hassouna<hassouna@cs.uni-bremen.de>
;;;               2020, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;               2020, Thomas Lipps <tlipps@uni-bremen.de>
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

;;;;;;;;;;;;;;;;;;;;;;;;;;;; BIG-KNIFE ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *big-knife-grasp-z-offset* -0.020 ;; 0.015
              "in meters") ; because TCP is not at the edge
(defparameter *big-knife-pregrasp-z-offset* 0.25 "in meters")
(defparameter *big-knife-pregrasp-xy-offset* 0.10 "in meters")

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms
    '(:big-knife) '(:left :right) :front
  :grasp-translation `(0.0 0.06 ,*cutlery-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets `(0.0 0.06 ,*cutlery-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.06 ,*cutlery-pregrasp-z-offset*)
  :lift-translation `(0.0 0.06 ,*cutlery-pregrasp-z-offset*)
  :2nd-lift-translation `(0.0 0.06 ,*cutlery-pregrasp-z-offset*))
  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;; BREAD;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defparameter *bread-grasp-z-offset* 0.0 ;; 0.015
              "in meters") ; because TCP is not at the edge
(defparameter *bread-pregrasp-z-offset* 0.18 "in meters")
(defparameter *bread-pregrasp-xy-offset* 0.10 "in meters")


;; Left-TOP grasp
(man-int:def-object-type-to-gripper-transforms '(:bread)
    '(:left :right) :left-top
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets `(0.1 0.0 0.085)
  :2nd-pregrasp-offsets `(0.1 0.0 0.002))

;; right-TOP grasp
(man-int:def-object-type-to-gripper-transforms '(:bread)
    '(:left :right) :right-top
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets `(-0.1 0.0 0.085)
  :2nd-pregrasp-offsets `(-0.1 0.0 0.002))

;; left hold
(man-int:def-object-type-to-gripper-transforms '(:bread)
    '(:left :right) :left-hold
  :grasp-translation `(0.1 0.0 ,*bread-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(0.1 0.0 ,*bread-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.1 0.0 ,*bread-pregrasp-z-offset*)
  :lift-translation `(0.1 0.0 ,*bread-pregrasp-z-offset*)
  :2nd-lift-translation `(0.1 0.0 ,*bread-pregrasp-z-offset*))

;; right-holdy
(man-int:def-object-type-to-gripper-transforms '(:bread)
    '(:left :right) :right-hold
  :grasp-translation `(-0.1 0.0 ,*bread-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation*
  :pregrasp-offsets `(-0.1 0.0 ,*bread-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(-0.1 0.0 ,*bread-pregrasp-z-offset*)
  :lift-translation `(-0.1 0.0 ,*bread-pregrasp-z-offset*)
  :2nd-lift-translation `(-0.1 0.0 ,*bread-pregrasp-z-offset*))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Weisswurst ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defparameter *weisswurst-grasp-z-offset* 0.0 ;; 0.015
              "in meters") ; because TCP is not at the edge
(defparameter *weisswurst-pregrasp-z-offset* 0.18 "in meters")
(defparameter *weisswurst-pregrasp-xy-offset* 0.10 "in meters")

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms '(:weisswurst)
    '(:left :right) :top
  :grasp-translation `(0.0 0.0 ,*weisswurst-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*weisswurst-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*weisswurst-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 ,*weisswurst-pregrasp-z-offset*)
  :2nd-lift-translation`(0.0 0.0 ,*weisswurst-pregrasp-z-offset*))

;; Slice left
(man-int:def-object-type-to-gripper-transforms '(:weisswurst)
    '(:left :right) :left-top
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets `(0.04 0.0 0.037)
  :2nd-pregrasp-offsets `(0.04 0.0 0.001))

;; slice-right
(man-int:def-object-type-to-gripper-transforms '(:weisswurst)
    '(:left :right) :right-top
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets `(-0.04 0.0 0.037)
  :2nd-pregrasp-offsets `(-0.04 0.0 0.001))

;; left hold-hold
(man-int:def-object-type-to-gripper-transforms '(:weisswurst)
    '(:left :right) :left-hold
  :grasp-translation `(0.04 0.0 ,*weisswurst-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(0.04 0.0 ,*weisswurst-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.04 0.0 ,*weisswurst-pregrasp-z-offset*)
  :lift-translation `(0.04 0.0 ,*weisswurst-pregrasp-z-offset*)
  :2nd-lift-translation `(0.04 0.0 ,*weisswurst-pregrasp-z-offset*))


;; right-hold
(man-int:def-object-type-to-gripper-transforms '(:weisswurst)
    '(:left :right) :right-hold
  :grasp-translation `(-0.04 0.0 ,*weisswurst-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation*
  :pregrasp-offsets `(-0.04 0.0 ,*weisswurst-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(-0.04 0.0 ,*weisswurst-pregrasp-z-offset*)
  :lift-translation `(-0.04 0.0 ,*weisswurst-pregrasp-z-offset*)
  :2nd-lift-translation `(-0.04 0.0 ,*weisswurst-pregrasp-z-offset*))
  
  
  
