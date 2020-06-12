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

(in-package :objects)

(defparameter *lift-z-offset* 0.15 "in meters")
(defparameter *lift-offset* `(0.0 0.0 ,*lift-z-offset*))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-fact-group household-object-type-hierarchy (man-int:object-type-direct-subtype)
  (<- (man-int:object-type-direct-subtype :household-item :cutlery))
  (<- (man-int:object-type-direct-subtype :household-item :plate))
  (<- (man-int:object-type-direct-subtype :household-item :tray))
  (<- (man-int:object-type-direct-subtype :household-item :bottle))
  (<- (man-int:object-type-direct-subtype :household-item :cup))
  (<- (man-int:object-type-direct-subtype :household-item :milk))
  (<- (man-int:object-type-direct-subtype :household-item :cereal))
  (<- (man-int:object-type-direct-subtype :household-item :bowl))

  (<- (man-int:object-type-direct-subtype :cutlery :knife))
  (<- (man-int:object-type-direct-subtype :cutlery :fork))
  (<- (man-int:object-type-direct-subtype :cutlery :spoon))

  (<- (man-int:object-type-direct-subtype :cereal :breakfast-cereal)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-action-gripping-effort :heuristics 20
    ((object-type (eql :household-item)))
  50)
(defmethod man-int:get-action-gripping-effort :heuristics 20
    ((object-type (eql :milk)))
  20)
(defmethod man-int:get-action-gripping-effort :heuristics 20
    ((object-type (eql :cereal)))
  30)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-action-gripper-opening :heuristics 20
    ((object-type (eql :household-item)))
  0.10)
(defmethod man-int:get-action-gripper-opening :heuristics 20
    ((object-type (eql :cutlery)))
  0.04)
(defmethod man-int:get-action-gripper-opening :heuristics 20
    ((object-type (eql :plate)))
  0.02)
(defmethod man-int:get-action-gripper-opening :heuristics 20
    ((object-type (eql :tray)))
  0.02)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-object-type-carry-config :heuristics 20 (object-type grasp)
  :carry)
(defmethod man-int:get-object-type-carry-config :heuristics 20
    ((object-type (eql :household-item)) grasp)
  :carry)
(defmethod man-int:get-object-type-carry-config :heuristics 20
    ((object-type (eql :bowl)) grasp)
  :carry-top)
(defmethod man-int:get-object-type-carry-config :heuristics 20
    ((object-type (eql :cup)) (grasp (eql :top)))
  :carry-top)
(defmethod man-int:get-object-type-carry-config :heuristics 20
    ((object-type (eql :cereal)) (grasp (eql :top)))
  :carry-top)
(defmethod man-int:get-object-type-carry-config :heuristics 20
    ((object-type (eql :plate)) grasp)
  :carry-side-gripper-vertical)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-fact-group pnp-object-knowledge (man-int:object-rotationally-symmetric
                                      man-int:orientation-matters)

  (<- (object-rotationally-symmetric ?object-type)
    (member ?object-type (;; :plate :bottle :drink :cup :bowl :milk
                                 )))

  (<- (orientation-matters ?object-type)
    (member ?object-type (:knife :fork :spoon :cutlery :spatula))))

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
  :lift-translation `(0.0 0.0 ,*cutlery-pregrasp-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*cutlery-pregrasp-z-offset*))

;; BOTTOM grasp
;; Bottom grasp is commented out because the robot grasps the spoon through the
;; drawer, as in the last part of the grasping trajectory collisions are turned off
;; (man-int:def-object-type-to-gripper-transforms '(:cutlery :fork :knife :spoon)
;;     '(:left :right) :bottom
;;   :grasp-translation `(0.0 0.0 ,(- *cutlery-grasp-z-offset*))
;;   :grasp-rot-matrix man-int:*-z-across-x-grasp-rotation*
;;   :pregrasp-offsets `(0.0 0.0 ,(- *cutlery-pregrasp-z-offset*))
;;   :2nd-pregrasp-offsets `(0.0 0.0 ,(- *cutlery-pregrasp-z-offset*))
;;   :lift-translation `(0.0 0.0 ,*cutlery-pregrasp-z-offset*)
;;   :2nd-lift-translation `(0.0 0.0 ,*cutlery-pregrasp-z-offset*))

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
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms '(:tray) :right :right-side
  :grasp-translation `(0.0 ,(- *plate-grasp-y-offset*) ,*plate-grasp-z-offset*)
  :grasp-rot-matrix
  `((0 -1 0)
    (,(- (sin *plate-grasp-roll-offset*)) 0 ,(cos *plate-grasp-roll-offset*))
    (,(- (cos *plate-grasp-roll-offset*)) 0 ,(- (sin *plate-grasp-roll-offset*))))
  :pregrasp-offsets `(0.0 ,(- *plate-pregrasp-y-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *plate-pregrasp-y-offset*) ,*plate-2nd-pregrasp-z-offset*)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :plate :right :right-side
  :grasp-translation `(0.0 ,(- *plate-grasp-y-offset*) ,*plate-grasp-z-offset*)
  :grasp-rot-matrix
  `((0             -1 0)
    (,(- (sin *plate-grasp-roll-offset*)) 0  ,(cos *plate-grasp-roll-offset*))
    (,(- (cos *plate-grasp-roll-offset*)) 0  ,(- (sin *plate-grasp-roll-offset*))))
  :pregrasp-offsets `(0.0 ,(- *plate-pregrasp-y-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *plate-pregrasp-y-offset*) ,*plate-2nd-pregrasp-z-offset*)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)


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
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms '(:drink :bottle) '(:left :right) :right-side
  :grasp-translation `(0.0d0 ,*bottle-grasp-xy-offset* ,*bottle-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *bottle-pregrasp-xy-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *bottle-pregrasp-xy-offset*) 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; BACK grasp
(man-int:def-object-type-to-gripper-transforms '(:drink :bottle) '(:left :right) :back
  :grasp-translation `(,(- *bottle-grasp-xy-offset*) 0.0d0 ,*bottle-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation*
  :pregrasp-offsets `(,(- *bottle-pregrasp-xy-offset*) 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,(- *bottle-pregrasp-xy-offset*) 0.0 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; FRONT grasp
(man-int:def-object-type-to-gripper-transforms '(:drink :bottle) '(:left :right) :front
  :grasp-translation `(,*bottle-grasp-xy-offset* 0.0d0 ,*bottle-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(,*bottle-pregrasp-xy-offset* 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,*bottle-pregrasp-xy-offset* 0.0 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)


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
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; SIDE grasp
(man-int:def-object-type-to-gripper-transforms :cup '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,(- *cup-grasp-xy-offset*) ,*cup-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*cup-pregrasp-xy-offset* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*cup-pregrasp-xy-offset* 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :cup '(:left :right) :right-side
  :grasp-translation `(0.0d0 ,*cup-grasp-xy-offset* ,*cup-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *cup-pregrasp-xy-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *cup-pregrasp-xy-offset*) 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; BACK grasp
(man-int:def-object-type-to-gripper-transforms :cup '(:left :right) :back
  :grasp-translation `(,*cup-grasp-xy-offset* 0.0d0 ,*cup-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation*
  :pregrasp-offsets `(,(- *cup-pregrasp-xy-offset*) 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,(- *cup-pregrasp-xy-offset*) 0.0 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; FRONT grasp
(man-int:def-object-type-to-gripper-transforms :cup '(:left :right) :front
  :grasp-translation `(,(- *cup-grasp-xy-offset*) 0.0d0 ,*cup-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(,*cup-pregrasp-xy-offset* 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,*cup-pregrasp-xy-offset* 0.0 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)


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
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; FRONT grasp
(man-int:def-object-type-to-gripper-transforms :milk '(:left :right) :front
  :grasp-translation `(,(- *milk-grasp-xy-offset*) 0.0d0 ,*milk-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(,*milk-pregrasp-xy-offset* 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,*milk-pregrasp-xy-offset* 0.0 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; SIDE grasp
(man-int:def-object-type-to-gripper-transforms :milk '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,(- *milk-grasp-xy-offset*) ,*milk-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*milk-pregrasp-xy-offset* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*milk-pregrasp-xy-offset* 0.0)
  :lift-translation `(0.0 ,(- *milk-grasp-xy-offset*) ,*milk-lift-z-offset*)
  :2nd-lift-translation `(0.0 ,(- *milk-grasp-xy-offset*) ,*milk-lift-z-offset*))

(man-int:def-object-type-to-gripper-transforms :milk '(:left :right) :right-side
  :grasp-translation `(0.0d0 ,*milk-grasp-xy-offset* ,*milk-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *milk-pregrasp-xy-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *milk-pregrasp-xy-offset*) 0.0)
  :lift-translation `(0.0 ,*milk-grasp-xy-offset* ,*milk-lift-z-offset*)
  :2nd-lift-translation `(0.0 ,*milk-grasp-xy-offset* ,*milk-lift-z-offset*))

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
  :lift-translation `(,*cereal-grasp-xy-offset* 0.0 ,*cereal-lift-z-offset*)
  :2nd-lift-translation `(,*cereal-postgrasp-xy-offset* 0.0 ,*cereal-lift-z-offset*))

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms '(:cereal :breakfast-cereal) '(:left :right) :top
  :grasp-translation `(0.0d0 0.0d0 ,*cereal-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; BACK grasp
(man-int:def-object-type-to-gripper-transforms '(:cereal :breakfast-cereal) '(:left :right) :back
  :grasp-translation `(,(- *cereal-grasp-xy-offset*) 0.0d0 ,*cereal-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation*
  :pregrasp-offsets `(,(- *cereal-pregrasp-xy-offset*) 0.0 ,*cereal-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(,(- *cereal-pregrasp-xy-offset*) 0.0 0.0)
  :lift-translation `(,(- *cereal-grasp-xy-offset*) 0.0 ,*cereal-lift-z-offset*)
  :2nd-lift-translation `(,(- *cereal-postgrasp-xy-offset*) 0.0 ,*cereal-lift-z-offset*))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;; bowl ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *bowl-grasp-x-offset* 0.07 "in meters")
(defparameter *bowl-grasp-z-offset* 0.0 "in meters")
(defparameter *bowl-pregrasp-z-offset* 0.20 "in meters")

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms :bowl '(:left :right) :top
  :grasp-translation `(,(- *bowl-grasp-x-offset*) 0.0d0 ,*bowl-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*bowl-pregrasp-z-offset*))
(man-int:def-object-type-to-gripper-transforms :bowl '(:left :right) :top-front
  :grasp-translation `(,*bowl-grasp-x-offset* 0.0d0 ,*bowl-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*bowl-pregrasp-z-offset*))
(man-int:def-object-type-to-gripper-transforms :bowl '(:left :right) :top-left
  :grasp-translation `(0.0d0 ,*bowl-grasp-x-offset* ,*bowl-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*bowl-pregrasp-z-offset*))
(man-int:def-object-type-to-gripper-transforms :bowl '(:left :right) :top-right
  :grasp-translation `(0.0d0 ,(- *bowl-grasp-x-offset*) ,*bowl-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*bowl-pregrasp-z-offset*))





;;;;;;;;;;;;;;;;;;;;;;;;; table setting locations ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun make-location-on-sink-left-front (?environment-name)
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name sink-area-surface)
                         (owl-name "kitchen_sink_block_counter_top")
                         (part-of ?environment-name)))
           (side left)
           (side front)
           (range-invert 0.5)))

(mapcar (lambda (type)
          (defmethod man-int:get-object-likely-location :heuristics 20
              ((object-type (eql type))
               environment human
               (context (eql :table-setting)))
            (make-location-on-sink-left-front environment)))
        '(:plate :bowl))

(defun make-location-on-sink-left (?environment-name)
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name sink-area-surface)
                         (owl-name "kitchen_sink_block_counter_top")
                         (part-of ?environment-name)))
           (side left)))

(mapcar (lambda (type)
          (defmethod man-int:get-object-likely-location :heuristics 20
              ((object-type (eql type))
               environment human
               (context (eql :table-setting)))
            (make-location-on-sink-left-front environment)))
        '(:mug :cup))

(defun make-location-on-sink-middle-front (?environment-name)
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name sink-area-surface)
                         (owl-name "kitchen_sink_block_counter_top")
                         (part-of ?environment-name)))
           (side left)
           (side front)
           (range 0.5)))

(mapcar (lambda (type)
          (defmethod man-int:get-object-likely-location :heuristics 20
              ((object-type (eql type))
               environment human
               (context (eql :table-setting)))
            (make-location-on-sink-middle-front environment)))
        '(:bottle :milk :cereal :breakfast-cereal))

(defun make-location-in-sink-left-middle-drawer (?environment-name)
  (desig:a location
           (in (desig:an object
                         (type drawer)
                         (urdf-name sink-area-left-middle-drawer-main)
                         (owl-name "drawer_sinkblock_middle_open")
                         (part-of ?environment-name)))))

;; (mapcar (lambda (type)
;;           (defmethod man-int:get-object-likely-location :heuristics 20
;;               ((object-type (eql type))
;;                environment human
;;                (context (eql :table-setting)))
;;             (make-location-in-sink-left-middle-drawer environment)))
;;         '(:bowl :cup))

(defun make-location-in-fridge (?environment-name)
  (desig:a location
           (in (desig:an object
                         (type fridge)
                         (urdf-name iai-fridge-main)
                         (owl-name "drawer_fridge_upper_interior")
                         (part-of ?environment-name)
                         (level topmost)))))

;; (mapcar (lambda (type)
;;           (defmethod man-int:get-object-likely-location :heuristics 20
;;               ((object-type (eql type))
;;                environment human
;;                (context (eql :table-setting)))
;;             (make-location-in-fridge environment)))
;;         '(:milk))

(defun make-location-in-oven-right-drawer (?environment-name)
  (desig:a location
           ;; (side front)
           (in (desig:an object
                         (type drawer)
                         (urdf-name oven-area-area-right-drawer-main)
                         (owl-name "drawer_oven_right_open")
                         (part-of ?environment-name)
                         (level topmost)))))

;; (mapcar (lambda (type)
;;           (defmethod man-int:get-object-likely-location :heuristics 20
;;               ((object-type (eql type))
;;                environment human
;;                (context (eql :table-setting)))
;;             (make-location-in-oven-right-drawer environment)))
;;         '(:cereal))

(defun make-location-in-sink-left-upper-drawer (?environment-name)
  (desig:a location
           (in (desig:an object
                         (type drawer)
                         (urdf-name sink-area-left-upper-drawer-main)
                         (owl-name "drawer_sinkblock_upper_open")
                         (part-of ?environment-name)))
           (side front)))

(mapcar (lambda (type)
          (defmethod man-int:get-object-likely-location :heuristics 20
              ((object-type (eql type))
               environment human
               (context (eql :table-setting)))
            (make-location-in-sink-left-upper-drawer environment)))
        '(:cutlery :spoon))

;;;; destination

(defun make-location-on-kitchen-island-slots (?object-type ?environment-name)
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name kitchen-island-surface)
                         (owl-name "kitchen_island_counter_top")
                         (part-of ?environment-name)))
           (for (desig:an object (type ?object-type)))
           (side back)
           (side right)
           (range-invert 0.5)
           (context table-setting)
           (object-count 3)))

(mapcar (lambda (object-type)
          (defmethod man-int:get-object-destination :heuristics 20
              ((object-type (eql object-type))
               environment human
               (context (eql :table-setting)))
            (make-location-on-kitchen-island-slots object-type environment)))
        '(:bowl :plate))

(defun make-location-on-kitchen-island (?object-type ?environment-name)
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name kitchen-island-surface)
                         (owl-name "kitchen_island_counter_top")
                         (part-of ?environment-name)))
           (for (desig:an object (type ?object-type)))))

(mapcar (lambda (object-type)
          (defmethod man-int:get-object-destination :heuristics 20
              ((object-type (eql object-type))
               environment human
               (context (eql :table-setting)))
            (make-location-on-kitchen-island object-type environment)))
        '(:mug :bottle))

(defun make-location-right-of-other-object (?object-type ?other-object-type)
  (desig:a location
           (right-of (desig:an object (type ?other-object-type)))
           (near (desig:an object (type ?other-object-type)))
           (for (desig:an object (type ?object-type)))
           (orientation support-aligned)))

(mapcar (lambda (object-type-and-other-object-type)
          (destructuring-bind (object-type other-object-type)
              object-type-and-other-object-type
            (defmethod man-int:get-object-destination :heuristics 20
                ((object-type (eql object-type))
                 environment human
                 (context (eql :table-setting)))
              (make-location-right-of-other-object object-type other-object-type))))
          '((:spoon :bowl)
            (:knife :plate)))

(defun make-location-right-of-behind-other-object (?object-type ?other-object-type)
  (desig:a location
           (right-of (desig:an object (type ?other-object-type)))
           ;; (behind (desig:an object (type ?other-object-type)))
           (near (desig:an object (type ?other-object-type)))
           (for (desig:an object (type ?object-type)))))

(mapcar (lambda (object-type-and-other-object-type)
          (destructuring-bind (object-type other-object-type)
              object-type-and-other-object-type
            (defmethod man-int:get-object-destination :heuristics 20
                ((object-type (eql object-type))
                 environment human
                 (context (eql :table-setting)))
              (make-location-right-of-behind-other-object object-type
                                                          other-object-type))))
        '((:cup :bowl)))

(defun make-cereal-pose (?object-type ?environment-name)
  (let ((?pose
          (cl-transforms-stamped:make-pose-stamped
           "map"
           0.0
           (cl-transforms:make-3d-vector -0.78 0.8 0.95)
           (cl-transforms:make-quaternion 0 0 0.6 0.4)))
        ;; (?other-object :bowl)
        )
    (desig:a location
             ;; (pose ?pose)
             (on (desig:an object
                           (type counter-top)
                           (urdf-name kitchen-island-surface)
                           (owl-name "kitchen_island_counter_top")
                           (part-of ?environment-name)))
             (for (desig:an object (type ?object-type)))
             (side back)
             (side right)
             ;; (orientation axis-aligned)
             ;; (left-of (desig:an object (type ?other-object)))
             ;; (far-from (desig:an object (type ?other-object)))
             )))

(defmethod man-int:get-object-destination :heuristics 20
    ((object-type (eql :cereal))
     environment-name human-name
     (context (eql :table-setting)))
  (make-cereal-pose object-type environment-name))

(defun make-location-left-of-far-other-object (?object-type ?other-object-type)
  (desig:a location
           (left-of (desig:an object (type ?other-object-type)))
           (far-from (desig:an object (type ?other-object-type)))
           (for (desig:an object (type ?object-type)))))

(defmethod man-int:get-object-destination :heuristics 20
    ((object-type (eql :milk))
     environment human
     (context (eql :table-setting)))
  (make-location-left-of-far-other-object object-type :bowl))

;;;;;;;;;;;;;;;;;;;;;;;; table cleaning ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; This does not work. First, call-with-most-specific requires
;; a method to be defined for the given object-type,
;; if object-type is :household-item, then it is also
;; forwarded as :household-item to get-object-destination,
;; which is wrong.
;; Finally, get-object-destination of :table-setting is
;; no the same as object-likely-location of :table-cleaning,
;; as during setting we are relative to another object,
;; and during cleaning that object might already be taken away.
;; (defmethod man-int:get-object-likely-location (object-type
;;                                                environment human
;;                                                (context (eql :table-cleaning)))
;;   (man-int:get-object-destination object-type environment human :table-setting))

(mapcar (lambda (type)
          (defmethod man-int:get-object-likely-location :heuristics 20
              ((object-type (eql type))
               environment human
               (context (eql :table-cleaning)))
            (make-location-on-kitchen-island environment object-type)))
        '(:household-item))

(defun make-location-on-sink (?environment-name ?object-type)
  (desig:a location
           (on (desig:an object
                         (type area-sink)
                         (urdf-name sink-area-sink)
                         (owl-name "kitchen_sink_area_sink")
                         (part-of ?environment-name)))
           (for (desig:an object (type ?object-type)))))

(mapcar (lambda (type)
          (defmethod man-int:get-object-destination :heuristics 20
              ((object-type (eql type))
               environment human
               (context (eql :table-cleaning)))
            (make-location-on-sink environment object-type)))
        '(:bowl :plate :cutlery :mug :cup :cereal :breakfast-cereal :milk :bottle))
