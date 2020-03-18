;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     Vanessa Hassouna <hassouna@bremen.de>
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

(defparameter *lift-z-offset* 0.15 "in meters")
(defparameter *lift-offset* `(0.0 0.0 ,*lift-z-offset*))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-fact-group household-object-type-hierarchy (man-int:object-type-direct-subtype)
  (<- (man-int:object-type-direct-subtype :household-item :weisswurst))
  (<- (man-int:object-type-direct-subtype :household-item :bread))
  
  (<- (man-int:object-type-direct-subtype :cutlery :knife))
  (<- (man-int:object-type-direct-subtype :cutlery :big-knife))
  (<- (man-int:object-type-direct-subtype :cutlery :fork))
  (<- (man-int:object-type-direct-subtype :cutlery :spoon))

  (<- (man-int:object-type-direct-subtype :cereal :breakfast-cereal)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-action-gripping-effort :heuristics 20 ((object-type (eql :household-item)))
  50)
(defmethod man-int:get-action-gripping-effort :heuristics 20 ((object-type (eql :milk)))
  20)
(defmethod man-int:get-action-gripping-effort :heuristics 20 ((object-type (eql :cereal)))
  30)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-action-gripper-opening :heuristics 20 ((object-type (eql :household-item)))
  0.10)
(defmethod man-int:get-action-gripper-opening :heuristics 20 ((object-type (eql :cutlery)))
  0.04)
(defmethod man-int:get-action-gripper-opening :heuristics 20 ((object-type (eql :plate)))
  0.02)
(defmethod man-int:get-action-gripper-opening :heuristics 20 ((object-type (eql :tray)))
  0.02)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-fact-group pnp-object-knowledge (man-int:object-rotationally-symmetric
                                      man-int:orientation-matters)

  (<- (object-rotationally-symmetric ?object-type)
    (member ?object-type (;; :plate :bottle :drink :cup :bowl :milk
                                 )))

  (<- (orientation-matters ?object-type)
    (member ?object-type (:knife :fork :spoon :cutlery :spatula :weisswurst :bread :big-knife))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;; CUTLERY ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *cutlery-grasp-z-offset* -0.015 ;; 0.015
              "in meters") ; because TCP is not at the edge
(defparameter *cutlery-pregrasp-z-offset* 0.20 "in meters")
(defparameter *cutlery-pregrasp-xy-offset* 0.10 "in meters")

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms '(:cutlery :fork :knife :spoon)
    '(:left :right) :top
  :grasp-translation `(0.0 0.0 ,*cutlery-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*cutlery-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*cutlery-pregrasp-z-offset*)
  :lift-offsets `(0.0 0.0 ,*cutlery-pregrasp-z-offset*)
  :2nd-lift-offsets `(0.0 0.0 ,*cutlery-pregrasp-z-offset*))

;; front grasp
(man-int:def-object-type-to-gripper-transforms '(:cutlery :fork :knife :spoon)
    '(:left :right) :front
  :grasp-translation `(-0.03 0.0 0.0)
  :grasp-rot-matrix man-int:*y-across-x-grasp-rotation*
  :pregrasp-offsets `(-0.03 0.0 0.0)
  :2nd-pregrasp-offsets `(-0.03 0.0 0.0)
  :lift-offsets `(-0.03 0.0 0.0)
  :2nd-lift-offsets `(-0.03 0.0 0.0))

;; BOTTOM grasp
;; Bottom grasp is commented out because the robot grasps the spoon through the
;; drawer, as in the last part of the grasping trajectory collisions are turned off
;; (man-int:def-object-type-to-gripper-transforms '(:cutlery :fork :knife :spoon)
;;     '(:left :right) :bottom
;;   :grasp-translation `(0.0 0.0 ,(- *cutlery-grasp-z-offset*))
;;   :grasp-rot-matrix man-int:*-z-across-x-grasp-rotation*
;;   :pregrasp-offsets `(0.0 0.0 ,(- *cutlery-pregrasp-z-offset*))
;;   :2nd-pregrasp-offsets `(0.0 0.0 ,(- *cutlery-pregrasp-z-offset*))
;;   :lift-offsets `(0.0 0.0 ,(- *cutlery-pregrasp-z-offset*))
;;   :2nd-lift-offsets `(0.0 0.0 ,(- *cutlery-pregrasp-z-offset*)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; PLATE ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *plate-diameter* 0.26 "in meters")
(defparameter *plate-grasp-y-offset* (- (/ *plate-diameter* 2) 0.015) "in meters")
(defparameter *plate-grasp-z-offset* 0.015 "in meters")
(defparameter *plate-grasp-roll-offset* (/ pi 6))
(defparameter *plate-pregrasp-y-offset* 0.2 "in meters")
(defparameter *plate-2nd-pregrasp-z-offset* 0.03 "in meters") ; grippers can't go into table

;; SIDE grasp
(man-int:def-object-type-to-gripper-transforms '(:plate :tray) :left :left-side
  :grasp-translation `(0.0 ,*plate-grasp-y-offset* ,*plate-grasp-z-offset*)
  :grasp-rot-matrix
  `((0             1 0)
    (,(sin *plate-grasp-roll-offset*)     0 ,(- (cos *plate-grasp-roll-offset*)))
    (,(- (cos *plate-grasp-roll-offset*)) 0 ,(- (sin *plate-grasp-roll-offset*))))
  :pregrasp-offsets `(0.0 ,*plate-pregrasp-y-offset* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*plate-pregrasp-y-offset* ,*plate-2nd-pregrasp-z-offset*)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)
(man-int:def-object-type-to-gripper-transforms :plate :right :right-side
  :grasp-translation `(0.0 ,(- *plate-grasp-y-offset*) ,*plate-grasp-z-offset*)
  :grasp-rot-matrix
  `((0             -1 0)
    (,(- (sin *plate-grasp-roll-offset*)) 0  ,(cos *plate-grasp-roll-offset*))
    (,(- (cos *plate-grasp-roll-offset*)) 0  ,(- (sin *plate-grasp-roll-offset*))))
  :pregrasp-offsets `(0.0 ,(- *plate-pregrasp-y-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *plate-pregrasp-y-offset*) ,*plate-2nd-pregrasp-z-offset*)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; bottle ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *bottle-pregrasp-xy-offset* 0.15 "in meters")
(defparameter *bottle-grasp-xy-offset* 0.02 "in meters")
(defparameter *bottle-grasp-z-offset* 0.005 "in meters")

;; SIDE grasp
(man-int:def-object-type-to-gripper-transforms '(:drink :bottle) '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,(- *bottle-grasp-xy-offset*) ,*bottle-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*bottle-pregrasp-xy-offset* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*bottle-pregrasp-xy-offset* 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)
(man-int:def-object-type-to-gripper-transforms '(:drink :bottle) '(:left :right) :right-side
  :grasp-translation `(0.0d0 ,*bottle-grasp-xy-offset* ,*bottle-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *bottle-pregrasp-xy-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *bottle-pregrasp-xy-offset*) 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

;; BACK grasp
(man-int:def-object-type-to-gripper-transforms '(:drink :bottle) '(:left :right) :back
  :grasp-translation `(,*bottle-grasp-xy-offset* 0.0d0 ,*bottle-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation*
  :pregrasp-offsets `(,(- *bottle-pregrasp-xy-offset*) 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,(- *bottle-pregrasp-xy-offset*) 0.0 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

;; FRONT grasp
(man-int:def-object-type-to-gripper-transforms '(:drink :bottle) '(:left :right) :front
  :grasp-translation `(,*bottle-grasp-xy-offset* 0.0d0 ,*bottle-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(,(- *bottle-pregrasp-xy-offset*) 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,(- *bottle-pregrasp-xy-offset*) 0.0 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;; cup ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *cup-pregrasp-xy-offset* 0.15 "in meters")
(defparameter *cup-grasp-xy-offset* 0.02 "in meters")
(defparameter *cup-grasp-z-offset* 0.01 "in meters")
(defparameter *cup-top-grasp-x-offset* 0.03 "in meters")
(defparameter *cup-top-grasp-z-offset* 0.02 "in meters")

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms :cup '(:left :right) :top
  :grasp-translation `(,(- *cup-top-grasp-x-offset*) 0.0d0 ,*cup-top-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

;; SIDE grasp
(man-int:def-object-type-to-gripper-transforms :cup '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,(- *cup-grasp-xy-offset*) ,*cup-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*cup-pregrasp-xy-offset* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*cup-pregrasp-xy-offset* 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*
  :tilt-approach-offsets '(0.0 0.085 0.065))

(man-int:def-object-type-to-gripper-transforms :cup '(:left :right) :right-side
  :grasp-translation `(0.0d0 ,*cup-grasp-xy-offset* ,*cup-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *cup-pregrasp-xy-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *cup-pregrasp-xy-offset*) 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*
  :tilt-approach-offsets '(0.0 -0.085 0.065))

;; BACK grasp
(man-int:def-object-type-to-gripper-transforms :cup '(:left :right) :back
  :grasp-translation `(,*cup-grasp-xy-offset* 0.0d0 ,*cup-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation*
  :pregrasp-offsets `(,(- *cup-pregrasp-xy-offset*) 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,(- *cup-pregrasp-xy-offset*) 0.0 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*
  :tilt-approach-offsets '(-0.085 0.0 0.065))

;; FRONT grasp
(man-int:def-object-type-to-gripper-transforms :cup '(:left :right) :left-side
  :grasp-translation `(,(- *cup-grasp-xy-offset*) 0.0d0 ,*cup-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(,*cup-pregrasp-xy-offset* 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,*cup-pregrasp-xy-offset* 0.0 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*
  :tilt-approach-offsets '(0.085 0.0 0.065))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; milk ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *milk-grasp-xy-offset* 0.01 "in meters")
(defparameter *milk-grasp-z-offset* 0.0 "in meters")
(defparameter *milk-pregrasp-xy-offset* 0.15 "in meters")
(defparameter *milk-lift-z-offset* 0.05 "in meters")

;; BACK grasp
(man-int:def-object-type-to-gripper-transforms :milk '(:left :right) :back
  :grasp-translation `(,*milk-grasp-xy-offset* 0.0d0 ,*milk-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation*
  :pregrasp-offsets `(,(- *milk-pregrasp-xy-offset*) 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,(- *milk-pregrasp-xy-offset*) 0.0 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

;; FRONT grasp
(man-int:def-object-type-to-gripper-transforms :milk '(:left :right) :front
  :grasp-translation `(,(- *milk-grasp-xy-offset*) 0.0d0 ,*milk-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(,*milk-pregrasp-xy-offset* 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,*milk-pregrasp-xy-offset* 0.0 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

;; SIDE grasp
(man-int:def-object-type-to-gripper-transforms :milk '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,(- *milk-grasp-xy-offset*) ,*milk-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*milk-pregrasp-xy-offset* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*milk-pregrasp-xy-offset* 0.0)
  :lift-offsets `(0.0 ,(- *milk-grasp-xy-offset*) ,*milk-lift-z-offset*)
  :2nd-lift-offsets `(0.0 ,(- *milk-grasp-xy-offset*) ,*milk-lift-z-offset*))
(man-int:def-object-type-to-gripper-transforms :milk '(:left :right) :right-side
  :grasp-translation `(0.0d0 ,*milk-grasp-xy-offset* ,*milk-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *milk-pregrasp-xy-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *milk-pregrasp-xy-offset*) 0.0)
  :lift-offsets `(0.0 ,*milk-grasp-xy-offset* ,*milk-lift-z-offset*)
  :2nd-lift-offsets `(0.0 ,*milk-grasp-xy-offset* ,*milk-lift-z-offset*))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; cereal ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *cereal-grasp-z-offset* 0.04 "in meters")
(defparameter *cereal-grasp-xy-offset* 0.03 "in meters")
(defparameter *cereal-pregrasp-z-offset* 0.05 "in meters")
(defparameter *cereal-pregrasp-xy-offset* 0.15 "in meters")
(defparameter *cereal-postgrasp-xy-offset* 0.40 "in meters")
(defparameter *cereal-lift-z-offset* 0.1 "in meters")

;; FRONT grasp
(man-int:def-object-type-to-gripper-transforms '(:cereal :breakfast-cereal) '(:left :right) :front
  :grasp-translation `(,*cereal-grasp-xy-offset* 0.0d0 ,*cereal-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(,*cereal-pregrasp-xy-offset* 0.0 ,*cereal-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(,*cereal-pregrasp-xy-offset* 0.0 0.0)
  :lift-offsets `(,*cereal-grasp-xy-offset* 0.0 ,*cereal-lift-z-offset*)
  :2nd-lift-offsets `(,*cereal-postgrasp-xy-offset* 0.0 ,*cereal-lift-z-offset*))

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms '(:cereal :breakfast-cereal) '(:left :right) :top
  :grasp-translation `(0.0d0 0.0d0 ,*cereal-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

;; BACK grasp
(man-int:def-object-type-to-gripper-transforms '(:cereal :breakfast-cereal) '(:left :right) :back
  :grasp-translation `(,(- *cereal-grasp-xy-offset*) 0.0d0 ,*cereal-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation*
  :pregrasp-offsets `(,(- *cereal-pregrasp-xy-offset*) 0.0 ,*cereal-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(,(- *cereal-pregrasp-xy-offset*) 0.0 0.0)
  :lift-offsets `(,(- *cereal-grasp-xy-offset*) 0.0 ,*cereal-lift-z-offset*)
  :2nd-lift-offsets `(,(- *cereal-postgrasp-xy-offset*) 0.0 ,*cereal-lift-z-offset*))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;; bowl ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *bowl-grasp-x-offset* 0.07 "in meters")
(defparameter *bowl-grasp-z-offset* 0.0 "in meters")
(defparameter *bowl-pregrasp-z-offset* 0.30 "in meters")

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms :bowl '(:left :right) :top
  :grasp-translation `(,(- *bowl-grasp-x-offset*) 0.0d0 ,*bowl-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :lift-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :2nd-lift-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*))

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
  :lift-offsets `(0.0 0.0 ,*weisswurst-pregrasp-z-offset*)
  :2nd-lift-offsets `(0.0 0.0 ,*weisswurst-pregrasp-z-offset*))

;; Left-TOP grasp
(man-int:def-object-type-to-gripper-transforms '(:weisswurst)
    '(:left :right) :left-top
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :slice-up-offsets `(0.04 0.0 0.037)
  :slice-down-offsets `(0.04 0.0 0.001))

;; Right-TOP grasp
(man-int:def-object-type-to-gripper-transforms '(:weisswurst)
    '(:left :right) :right-top
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :slice-up-offsets `(-0.04 0.0 0.037)
  :slice-down-offsets `(-0.04 0.0 0.001))

;; left hold-hold
(man-int:def-object-type-to-gripper-transforms '(:weisswurst)
    '(:left :right) :left-hold
  :grasp-translation `(0.04 0.0 ,*weisswurst-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(0.04 0.0 ,*weisswurst-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.04 0.0 ,*weisswurst-pregrasp-z-offset*)
  :lift-offsets `(0.04 0.0 ,*weisswurst-pregrasp-z-offset*)
  :2nd-lift-offsets `(0.04 0.0 ,*weisswurst-pregrasp-z-offset*))


;; right-holdy
(man-int:def-object-type-to-gripper-transforms '(:weisswurst)
    '(:left :right) :right-hold
  :grasp-translation `(-0.04 0.0 ,*weisswurst-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation*
  :pregrasp-offsets `(-0.04 0.0 ,*weisswurst-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(-0.04 0.0 ,*weisswurst-pregrasp-z-offset*)
  :lift-offsets `(-0.04 0.0 ,*weisswurst-pregrasp-z-offset*)
  :2nd-lift-offsets `(-0.04 0.0 ,*weisswurst-pregrasp-z-offset*))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;; BREAD;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defparameter *bread-grasp-z-offset* 0.0 ;; 0.015
              "in meters") ; because TCP is not at the edge
(defparameter *bread-pregrasp-z-offset* 0.18 "in meters")
(defparameter *bread-pregrasp-xy-offset* 0.10 "in meters")


;; Left-TOP grasp
(man-int:def-object-type-to-gripper-transforms '(:bread)
    '(:left :right) :left-top
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :slice-up-offsets `(0.1 0.0 0.085)
  :slice-down-offsets `(0.1 0.0 0.002))

;; right-TOP grasp
(man-int:def-object-type-to-gripper-transforms '(:bread)
    '(:left :right) :right-top
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :slice-up-offsets `(-0.1 0.0 0.085)
  :slice-down-offsets `(-0.1 0.0 0.002))

;; left hold
(man-int:def-object-type-to-gripper-transforms '(:bread)
    '(:left :right) :left-hold
  :grasp-translation `(0.1 0.0 ,*bread-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(0.1 0.0 ,*bread-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.1 0.0 ,*bread-pregrasp-z-offset*)
  :lift-offsets `(0.1 0.0 ,*bread-pregrasp-z-offset*)
  :2nd-lift-offsets `(0.1 0.0 ,*bread-pregrasp-z-offset*))

;; right-holdy
(man-int:def-object-type-to-gripper-transforms '(:bread)
    '(:left :right) :right-hold
  :grasp-translation `(-0.1 0.0 ,*bread-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation*
  :pregrasp-offsets `(-0.1 0.0 ,*bread-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(-0.1 0.0 ,*bread-pregrasp-z-offset*)
  :lift-offsets `(-0.1 0.0 ,*bread-pregrasp-z-offset*)
  :2nd-lift-offsets `(-0.1 0.0 ,*bread-pregrasp-z-offset*))


;;;;;;;;;;;;;;;;;;;;;;;;;;;; BIG-KNIFE ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *big-knife-grasp-z-offset* -0.020 ;; 0.015
              "in meters") ; because TCP is not at the edge
(defparameter *big-knife-pregrasp-z-offset* 0.25 "in meters")
(defparameter *big-knife-pregrasp-xy-offset* 0.10 "in meters")

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms '(:big-knife)
    '(:left :right) :front
  :grasp-translation `(0.0 0.06 ,*cutlery-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets `(0.0 0.06 ,*cutlery-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.06 ,*cutlery-pregrasp-z-offset*)
  :lift-offsets `(0.0 0.06 ,*cutlery-pregrasp-z-offset*)
  :2nd-lift-offsets `(0.0 0.06 ,*cutlery-pregrasp-z-offset*))



