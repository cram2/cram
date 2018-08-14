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

(in-package :kr-pp)

(defparameter *lift-z-offset* 0.15 "in meters")
(defparameter *lift-offset* `(0.0 0.0 ,*lift-z-offset*))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod get-object-type-gripping-effort (object-type) 35) ; default in Nm
(defmethod get-object-type-gripping-effort ((object-type (eql :cutlery))) 100)
(defmethod get-object-type-gripping-effort ((object-type (eql :spoon))) 100)
(defmethod get-object-type-gripping-effort ((object-type (eql :fork))) 100)
(defmethod get-object-type-gripping-effort ((object-type (eql :knife))) 100)
(defmethod get-object-type-gripping-effort ((object-type (eql :plate))) 100)
(defmethod get-object-type-gripping-effort ((object-type (eql :tray))) 100)
(defmethod get-object-type-gripping-effort ((object-type (eql :bottle))) 60)
(defmethod get-object-type-gripping-effort ((object-type (eql :cup))) 50)
(defmethod get-object-type-gripping-effort ((object-type (eql :milk))) 15)
(defmethod get-object-type-gripping-effort ((object-type (eql :cereal))) 15)
(defmethod get-object-type-gripping-effort ((object-type (eql :breakfast-cereal))) 20)
(defmethod get-object-type-gripping-effort ((object-type (eql :bowl))) 100)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod get-object-type-gripper-opening (object-type) 0.10) ; default in meters
(defmethod get-object-type-gripper-opening ((object-type (eql :cutlery))) 0.04)
(defmethod get-object-type-gripper-opening ((object-type (eql :spoon))) 0.04)
(defmethod get-object-type-gripper-opening ((object-type (eql :fork))) 0.04)
(defmethod get-object-type-gripper-opening ((object-type (eql :knife))) 0.04)
(defmethod get-object-type-gripper-opening ((object-type (eql :plate))) 0.02)
(defmethod get-object-type-gripper-opening ((object-type (eql :tray))) 0.02)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-fact-group pnp-object-knowledge (object-rotationally-symmetric
                                      orientation-matters
                                      object-type-grasp)

  (<- (object-rotationally-symmetric ?object-type)
    (member ?object-type (;; :plate :bottle :drink :cup :bowl :milk
                                 )))

  (<- (orientation-matters ?object-type)
    (member ?object-type (:knife :fork :spoon :cutlery :spatula)))

  (<- (object-type-grasp :cutlery :top))
  (<- (object-type-grasp :spoon :top))
  (<- (object-type-grasp :fork :top))
  (<- (object-type-grasp :knife :top))

  (<- (object-type-grasp :plate :left-side))
  (<- (object-type-grasp :plate :right-side))
  (<- (object-type-grasp :tray :left-side))
  (<- (object-type-grasp :tray :right-side))

  (<- (object-type-grasp :bottle :left-side))
  (<- (object-type-grasp :bottle :right-side))
  (<- (object-type-grasp :bottle :back))
  (<- (object-type-grasp :bottle :front))

  (<- (object-type-grasp :cup :back))
  (<- (object-type-grasp :cup :front))
  (<- (object-type-grasp :cup :left-side))
  (<- (object-type-grasp :cup :right-side))
  ;; (<- (object-type-grasp :cup :top))

  (<- (object-type-grasp :milk :back))
  (<- (object-type-grasp :milk :left-side))
  (<- (object-type-grasp :milk :right-side))
  (<- (object-type-grasp :milk :front))
  ;; (<- (object-type-grasp :milk :top))

  ;; (<- (object-type-grasp :cereal :top))
  (<- (object-type-grasp :cereal :back))
  (<- (object-type-grasp :cereal :front))
  ;; (<- (object-type-grasp :breakfast-cereal :top))
  (<- (object-type-grasp :breakfast-cereal :back))
  (<- (object-type-grasp :breakfast-cereal :front))

  (<- (object-type-grasp :bowl :top)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;; CUTLERY ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *cutlery-grasp-z-offset* -0.0 "in meters") ; because TCP is not at the edge
(defparameter *cutlery-pregrasp-z-offset* 0.20 "in meters")
(defparameter *cutlery-pregrasp-xy-offset* 0.10 "in meters")

;; TOP grasp
(def-object-type-to-gripper-transforms '(:cutlery :spoon :fork :knife) '(:left :right) :top
  :grasp-translation `(0.0 0.0 ,*cutlery-grasp-z-offset*)
  :grasp-rot-matrix *top-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*cutlery-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*cutlery-pregrasp-z-offset*)
  :lift-offsets `(0.0 0.0 ,*cutlery-pregrasp-z-offset*)
  :2nd-lift-offsets `(0.0 0.0 ,*cutlery-pregrasp-z-offset*))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; PLATE ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *plate-diameter* 0.26 "in meters")
(defparameter *plate-grasp-y-offset* (- (/ *plate-diameter* 2) 0.015) "in meters")
(defparameter *plate-grasp-z-offset* 0.015 "in meters")
(defparameter *plate-grasp-roll-offset* (/ pi 6))
(defparameter *plate-pregrasp-y-offset* 0.2 "in meters")
(defparameter *plate-2nd-pregrasp-z-offset* 0.03 "in meters") ; grippers can't go into table

;; SIDE grasp
(def-object-type-to-gripper-transforms '(:plate :tray) :left :left-side
  :grasp-translation `(0.0 ,*plate-grasp-y-offset* ,*plate-grasp-z-offset*)
  :grasp-rot-matrix
  `((0             1 0)
    (,(sin *plate-grasp-roll-offset*)     0 ,(- (cos *plate-grasp-roll-offset*)))
    (,(- (cos *plate-grasp-roll-offset*)) 0 ,(- (sin *plate-grasp-roll-offset*))))
  :pregrasp-offsets `(0.0 ,*plate-pregrasp-y-offset* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*plate-pregrasp-y-offset* ,*plate-2nd-pregrasp-z-offset*)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)
(def-object-type-to-gripper-transforms :plate :right :right-side
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
(def-object-type-to-gripper-transforms '(:drink :bottle) '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,(- *bottle-grasp-xy-offset*) ,*bottle-grasp-z-offset*)
  :grasp-rot-matrix *left-side-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*bottle-pregrasp-xy-offset* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*bottle-pregrasp-xy-offset* 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)
(def-object-type-to-gripper-transforms '(:drink :bottle) '(:left :right) :right-side
  :grasp-translation `(0.0d0 ,*bottle-grasp-xy-offset* ,*bottle-grasp-z-offset*)
  :grasp-rot-matrix *right-side-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *bottle-pregrasp-xy-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *bottle-pregrasp-xy-offset*) 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

;; BACK grasp
(def-object-type-to-gripper-transforms '(:drink :bottle) '(:left :right) :back
  :grasp-translation `(,*bottle-grasp-xy-offset* 0.0d0 ,*bottle-grasp-z-offset*)
  :grasp-rot-matrix *back-grasp-rotation*
  :pregrasp-offsets `(,(- *bottle-pregrasp-xy-offset*) 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,(- *bottle-pregrasp-xy-offset*) 0.0 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

;; FRONT grasp
(def-object-type-to-gripper-transforms '(:drink :bottle) '(:left :right) :front
  :grasp-translation `(,*bottle-grasp-xy-offset* 0.0d0 ,*bottle-grasp-z-offset*)
  :grasp-rot-matrix *front-grasp-rotation*
  :pregrasp-offsets `(,(- *bottle-pregrasp-xy-offset*) 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,(- *bottle-pregrasp-xy-offset*) 0.0 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;; cup ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *cup-pregrasp-xy-offset* 0.15 "in meters")
(defparameter *cup-grasp-xy-offset* 0.02 "in meters")
(defparameter *cup-grasp-z-offset* 0.03 "in meters")
(defparameter *cup-top-grasp-x-offset* 0.03 "in meters")
(defparameter *cup-top-grasp-z-offset* 0.02 "in meters")

;; TOP grasp
(def-object-type-to-gripper-transforms :cup '(:left :right) :top
  :grasp-translation `(,(- *cup-top-grasp-x-offset*) 0.0d0 ,*cup-top-grasp-z-offset*)
  :grasp-rot-matrix *top-across-y-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

;; SIDE grasp
(def-object-type-to-gripper-transforms :cup '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,(- *cup-grasp-xy-offset*) ,*cup-grasp-z-offset*)
  :grasp-rot-matrix *left-side-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*cup-pregrasp-xy-offset* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*cup-pregrasp-xy-offset* 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)
(def-object-type-to-gripper-transforms :cup '(:left :right) :right-side
  :grasp-translation `(0.0d0 ,*cup-grasp-xy-offset* ,*cup-grasp-z-offset*)
  :grasp-rot-matrix *right-side-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *cup-pregrasp-xy-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *cup-pregrasp-xy-offset*) 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

;; BACK grasp
(def-object-type-to-gripper-transforms :cup '(:left :right) :back
  :grasp-translation `(,*cup-grasp-xy-offset* 0.0d0 ,*cup-grasp-z-offset*)
  :grasp-rot-matrix *back-grasp-rotation*
  :pregrasp-offsets `(,(- *cup-pregrasp-xy-offset*) 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,(- *cup-pregrasp-xy-offset*) 0.0 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

;; FRONT grasp
(def-object-type-to-gripper-transforms :cup '(:left :right) :front
  :grasp-translation `(,(- *cup-grasp-xy-offset*) 0.0d0 ,*cup-grasp-z-offset*)
  :grasp-rot-matrix *front-grasp-rotation*
  :pregrasp-offsets `(,*cup-pregrasp-xy-offset* 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,*cup-pregrasp-xy-offset* 0.0 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; milk ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *milk-grasp-xy-offset* 0.01 "in meters")
(defparameter *milk-grasp-z-offset* 0.0 "in meters")
(defparameter *milk-pregrasp-xy-offset* 0.15 "in meters")

;; SIDE grasp
(def-object-type-to-gripper-transforms :milk '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,(- *milk-grasp-xy-offset*) ,*milk-grasp-z-offset*)
  :grasp-rot-matrix *left-side-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*milk-pregrasp-xy-offset* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*milk-pregrasp-xy-offset* 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)
(def-object-type-to-gripper-transforms :milk '(:left :right) :right-side
  :grasp-translation `(0.0d0 ,*milk-grasp-xy-offset* ,*milk-grasp-z-offset*)
  :grasp-rot-matrix *right-side-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *milk-pregrasp-xy-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *milk-pregrasp-xy-offset*) 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

;; BACK grasp
(def-object-type-to-gripper-transforms :milk '(:left :right) :back
  :grasp-translation `(,*milk-grasp-xy-offset* 0.0d0 ,*milk-grasp-z-offset*)
  :grasp-rot-matrix *back-grasp-rotation*
  :pregrasp-offsets `(,(- *milk-pregrasp-xy-offset*) 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,(- *milk-pregrasp-xy-offset*) 0.0 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

;; FRONT grasp
(def-object-type-to-gripper-transforms :milk '(:left :right) :front
  :grasp-translation `(,(- *milk-grasp-xy-offset*) 0.0d0 ,*milk-grasp-z-offset*)
  :grasp-rot-matrix *front-grasp-rotation*
  :pregrasp-offsets `(,*milk-pregrasp-xy-offset* 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,*milk-pregrasp-xy-offset* 0.0 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; cereal ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *cereal-grasp-z-offset* 0.04 "in meters")
(defparameter *cereal-grasp-xy-offset* 0.03 "in meters")
(defparameter *cereal-pregrasp-xy-offset* 0.15 "in meters")

;; TOP grasp
(def-object-type-to-gripper-transforms '(:breakfast-cereal :cereal) '(:left :right) :top
  :grasp-translation `(0.0d0 0.0d0 ,*cereal-grasp-z-offset*)
  :grasp-rot-matrix *top-across-x-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

;; BACK grasp
(def-object-type-to-gripper-transforms '(:breakfast-cereal :cereal) '(:left :right) :back
  :grasp-translation `(,(- *cereal-grasp-xy-offset*) 0.0d0 ,*cereal-grasp-z-offset*)
  :grasp-rot-matrix *back-grasp-rotation*
  :pregrasp-offsets `(,(- *cereal-pregrasp-xy-offset*) 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,(- *cereal-pregrasp-xy-offset*) 0.0 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

;; FRONT grasp
(def-object-type-to-gripper-transforms '(:breakfast-cereal :cereal) '(:left :right) :front
  :grasp-translation `(,*cereal-grasp-xy-offset* 0.0d0 ,*cereal-grasp-z-offset*)
  :grasp-rot-matrix *front-grasp-rotation*
  :pregrasp-offsets `(,*cereal-pregrasp-xy-offset* 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,*cereal-pregrasp-xy-offset* 0.0 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;; bowl ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *bowl-grasp-x-offset* 0.07 "in meters")
(defparameter *bowl-grasp-z-offset* 0.01 "in meters")
(defparameter *bowl-pregrasp-z-offset* 0.20 "in meters")

;; TOP grasp
(def-object-type-to-gripper-transforms :bowl '(:left :right) :top
  :grasp-translation `(,(- *bowl-grasp-x-offset*) 0.0d0 ,*bowl-grasp-z-offset*)
  :grasp-rot-matrix *top-across-y-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :lift-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :2nd-lift-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*))
