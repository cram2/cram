
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

(defparameter *lift-z-offset* 0.07 "in meters")
(defparameter *lift-offset* `(0.0 0.0 ,*lift-z-offset*))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-fact-group household-object-type-hierarchy (man-int:object-type-direct-subtype)

  (<- (man-int:object-type-direct-subtype :household-item :kitchen-item))
  (<- (man-int:object-type-direct-subtype :household-item :clothing-item))

  (<- (man-int:object-type-direct-subtype :kitchen-item :cutlery))
  (<- (man-int:object-type-direct-subtype :kitchen-item :plate))
  (<- (man-int:object-type-direct-subtype :kitchen-item :tray))
  (<- (man-int:object-type-direct-subtype :kitchen-item :bottle))
  (<- (man-int:object-type-direct-subtype :kitchen-item :cup))
  (<- (man-int:object-type-direct-subtype :kitchen-item :milk))
  (<- (man-int:object-type-direct-subtype :kitchen-item :cereal))
  (<- (man-int:object-type-direct-subtype :kitchen-item :bowl))
  (<- (man-int:object-type-direct-subtype :kitchen-item :pot))
  (<- (man-int:object-type-direct-subtype :kitchen-item :spatula))


  (<- (man-int:object-type-direct-subtype :food :bread))
  (<- (man-int:object-type-direct-subtype :food :weisswurst))
  

  (<- (man-int:object-type-direct-subtype :cutlery :knife))
  (<- (man-int:object-type-direct-subtype :cutlery :big-knife))
  (<- (man-int:object-type-direct-subtype :cutlery :fork))
  (<- (man-int:object-type-direct-subtype :cutlery :spoon))

  (<- (man-int:object-type-direct-subtype :plate :ikea-small-plate))
  (<- (man-int:object-type-direct-subtype :plate :ikea-big-plate))

  (<- (man-int:object-type-direct-subtype :bowl :bowl-round))

  (<- (man-int:object-type-direct-subtype :cereal :breakfast-cereal))

  (<- (man-int:object-type-direct-subtype :cup :mug))
  (<- (man-int:object-type-direct-subtype :cup :jeroen-cup))

  (<- (man-int:object-type-direct-subtype :clothing-item :shoe)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-action-gripping-effort :heuristics 20
    ((object-type (eql :household-item)))
  50)
(defmethod man-int:get-action-gripping-effort :heuristics 20
    ((object-type (eql :food)))
  50)
(defmethod man-int:get-action-gripping-effort :heuristics 20
    ((object-type (eql :milk)))
  20)
(defmethod man-int:get-action-gripping-effort :heuristics 20
    ((object-type (eql :cereal)))
  45)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-action-gripper-opening :heuristics 20
    ((object-type (eql :household-item)))
  0.10)
(defmethod man-int:get-action-gripper-opening :heuristics 20
    ((object-type (eql :food)))
  0.10)
(defmethod man-int:get-action-gripper-opening :heuristics 20
    ((object-type (eql :cutlery)))
  0.05)
(defmethod man-int:get-action-gripper-opening :heuristics 20
    ((object-type (eql :plate)))
  0.02)
(defmethod man-int:get-action-gripper-opening :heuristics 20
    ((object-type (eql :ikea-small-plate)))
  0.03)
(defmethod man-int:get-action-gripper-opening :heuristics 20
    ((object-type (eql :ikea-big-plate)))
  0.03)
(defmethod man-int:get-action-gripper-opening :heuristics 20
    ((object-type (eql :tray)))
  0.02)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-object-type-carry-config :heuristics 20
    ((object-type (eql :household-item)) grasp)
  :carry)
(defmethod man-int:get-object-type-carry-config :heuristics 20
    ((object-type (eql :bowl)) grasp)
  :carry-top)
(defmethod man-int:get-object-type-carry-config :heuristics 20
    ((object-type (eql :ikea-bowl-ww)) grasp)
  :carry-top)
(defmethod man-int:get-object-type-carry-config :heuristics 20
    ((object-type (eql :cup)) (grasp (eql :top)))
  :carry-top)
(defmethod man-int:get-object-type-carry-config :heuristics 20
    ((object-type (eql :cereal)) (grasp (eql :top)))
  :carry-top)
(defmethod man-int:get-object-type-carry-config :heuristics 20
    ((object-type (eql :basket)) (grasp (eql :top)))
  :carry-top-basket)
(defmethod man-int:get-object-type-carry-config :heuristics 20
    ((object-type (eql :tray)) grasp)
  :carry-tray)
(defmethod man-int:get-object-type-carry-config :heuristics 20
    ((object-type (eql :plate)) grasp)
  :carry-side-gripper-vertical)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-arms-for-object-type :heuristics 20 ((object-type (eql :tray)))
  '(:left :right))
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-fact-group pnp-object-knowledge (man-int:object-rotationally-symmetric
                                      man-int:orientation-matters)

  (<- (object-rotationally-symmetric ?object-type)
    (member ?object-type (;; :plate :bottle :drink :cup :bowl :milk
                                 )))

  (<- (orientation-matters ?object-type)
    (member ?object-type (:knife :fork :spoon :cutlery :spatula :weisswurst
			  :bread :big-knife))))

(def-fact-group attachment-knowledge (man-int:unidirectional-attachment)

  (<- (man-int:unidirectional-attachment ?attachment-type)
    (member ?attachment-type (:popcorn-pot-lid-attachment))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;; CUTLERY ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *cutlery-grasp-z-offset* -0.015 ;; 0.015
              "in meters") ; because TCP is not at the edge
(defparameter *cutlery-pregrasp-z-offset* 0.20 "in meters")
(defparameter *cutlery-shelf-pregrasp-z-offset* 0.07 "in meters")
(defparameter *cutlery-pregrasp-xy-offset* 0.10 "in meters")

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms
    '(:cutlery :fork :knife :spoon) '(:left :right) :top
  :grasp-translation `(0.0 0.0 ,*cutlery-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*cutlery-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*cutlery-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 ,*cutlery-pregrasp-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*cutlery-pregrasp-z-offset*))
;; TOP grasp shelf
(man-int:def-object-type-to-gripper-transforms
    '(:cutlery :fork :knife :spoon) '(:left :right) :top
  :location-type :shelf
  :grasp-translation `(0.0 0.0 ,*cutlery-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*cutlery-shelf-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*cutlery-shelf-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 ,*cutlery-shelf-pregrasp-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*cutlery-shelf-pregrasp-z-offset*))

;; front grasp
(man-int:def-object-type-to-gripper-transforms '(:cutlery :fork :knife :spoon)
    '(:left :right) :front
  :grasp-translation `(-0.03 0.0 0.0)
  :grasp-rot-matrix man-int:*y-across-x-grasp-rotation*
  :pregrasp-offsets `(-0.03 0.0 0.0)
  :2nd-pregrasp-offsets `(-0.03 0.0 0.0)
  :lift-translation `(-0.03 0.0 0.0)
  :2nd-lift-translation `(-0.03 0.0 0.0))

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

;;;;;;;;;;;;;;;;;;;;;;;;;;;; SPATULA ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *spatula-grasp-z-offset* 0.0 "in meters") ; because TCP is not at the edge

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms :spatula '(:left :right) :top
  :grasp-translation `(0.0 0.0 ,*spatula-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*cutlery-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*cutlery-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 ,*cutlery-pregrasp-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*cutlery-pregrasp-z-offset*))
;; TOP grasp shelf
(man-int:def-object-type-to-gripper-transforms :spatula '(:left :right) :top
  :location-type :shelf
  :grasp-translation `(0.0 0.0 ,*spatula-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*cutlery-shelf-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*cutlery-shelf-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 ,*cutlery-shelf-pregrasp-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*cutlery-shelf-pregrasp-z-offset*))

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

(man-int:def-object-type-to-gripper-transforms '(:plate :tray) :right :right-side
  :grasp-translation `(0.0 ,(- *plate-grasp-y-offset*) ,*plate-grasp-z-offset*)
  :grasp-rot-matrix
  `((0 -1 0)
    (,(- (sin *plate-grasp-roll-offset*)) 0 ,(cos *plate-grasp-roll-offset*))
    (,(- (cos *plate-grasp-roll-offset*)) 0 ,(- (sin *plate-grasp-roll-offset*))))
  :pregrasp-offsets `(0.0 ,(- *plate-pregrasp-y-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *plate-pregrasp-y-offset*) ,*plate-2nd-pregrasp-z-offset*)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; IKEA-SMALL-PLATE ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *ikea-small-plate-diameter* 0.2 "in meters")
(defparameter *ikea-small-plate-edge* 0.04 "in meters")
(defparameter *ikea-small-plate-grasp-xy-offset*
  (- (/ *ikea-small-plate-diameter* 2) 0.02) "in meters")
(defparameter *ikea-small-plate-grasp-z-offset* 0.0 "in meters")
(defparameter *ikea-small-plate-pregrasp-xy-offset* 0.15 "in meters")
(defparameter *ikea-small-plate-pregrasp-z-offset*
  (* (tan (/ pi 6))
     (+ *ikea-small-plate-pregrasp-xy-offset* *ikea-small-plate-edge*))
  "in meters")
(defparameter *ikea-small-plate-2nd-pregrasp-xy-offset* 0.05 "in meters")
(defparameter *ikea-small-plate-2nd-pregrasp-z-offset*
  (* (tan (/ pi 6))
     (+ *ikea-small-plate-2nd-pregrasp-xy-offset* *ikea-small-plate-edge*))
  "in meters")
(defparameter *ikea-small-plate-z-offsets* '(0 0 0.10) "in meters")
(defparameter *ikea-small-plate-2nd-z-offsets* '(0 0 0.20) "in meters")

(man-int:def-object-type-to-gripper-transforms :ikea-small-plate '(:left :right) :left-side
  :grasp-translation `(0.0 ,*ikea-small-plate-grasp-xy-offset* ,*ikea-small-plate-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*ikea-small-plate-pregrasp-xy-offset* 0.0)
  :2nd-pregrasp-offsets `(0.0 ,*ikea-small-plate-2nd-pregrasp-xy-offset* 0.0)
  :lift-translation *ikea-small-plate-z-offsets*
  :2nd-lift-translation *ikea-small-plate-2nd-z-offsets*)

(man-int:def-object-type-to-gripper-transforms :ikea-small-plate '(:left :right) :right-side
  :grasp-translation `(0.0 ,(- *ikea-small-plate-grasp-xy-offset*) ,*ikea-small-plate-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *ikea-small-plate-pregrasp-xy-offset*) 0.0)
  :2nd-pregrasp-offsets `(0.0 ,(- *ikea-small-plate-2nd-pregrasp-xy-offset*) 0.0)
  :lift-translation *ikea-small-plate-z-offsets*
  :2nd-lift-translation *ikea-small-plate-2nd-z-offsets*)

(man-int:def-object-type-to-gripper-transforms :ikea-small-plate '(:left :right) :front
  :grasp-translation `(,*ikea-small-plate-grasp-xy-offset* 0.0 ,*ikea-small-plate-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-y-30-deg-grasp-rotation*
  :pregrasp-offsets `(,*ikea-small-plate-pregrasp-xy-offset*
                      0.0
                      ,*ikea-small-plate-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(,*ikea-small-plate-2nd-pregrasp-xy-offset*
                          0.0
                          ,*ikea-small-plate-2nd-pregrasp-z-offset*)
  :lift-translation *ikea-small-plate-z-offsets*
  :2nd-lift-translation *ikea-small-plate-2nd-z-offsets*)

(man-int:def-object-type-to-gripper-transforms :ikea-small-plate '(:left :right) :back
  :grasp-translation `(,(- *ikea-small-plate-grasp-xy-offset*) 0.0 ,*ikea-small-plate-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-y-grasp-rotation*
  :pregrasp-offsets `(,(- *ikea-small-plate-pregrasp-xy-offset*) 0.0 0.0)
  :2nd-pregrasp-offsets `(,(- *ikea-small-plate-2nd-pregrasp-xy-offset*) 0.0 0.0)
  :lift-translation *ikea-small-plate-z-offsets*
  :2nd-lift-translation *ikea-small-plate-2nd-z-offsets*)



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; IKEA-BIG-PLATE ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *ikea-big-plate-diameter* 0.263 "in meters")
(defparameter *ikea-big-plate-edge* 0.05 "in meters")
(defparameter *ikea-big-plate-grasp-xy-offset*
  (- (/ *ikea-big-plate-diameter* 2) 0.02) "in meters")
(defparameter *ikea-big-plate-grasp-z-offset* 0.0 "in meters")
(defparameter *ikea-big-plate-pregrasp-xy-offset* 0.15 "in meters")
(defparameter *ikea-big-plate-pregrasp-z-offset*
  (* (tan (/ pi 7.5))
     (+ *ikea-big-plate-pregrasp-xy-offset* *ikea-big-plate-edge*))
  "in meters")
(defparameter *ikea-big-plate-2nd-pregrasp-xy-offset* 0.05 "in meters")
(defparameter *ikea-big-plate-2nd-pregrasp-z-offset*
  (* (/ 2 5)
     (+ *ikea-big-plate-2nd-pregrasp-xy-offset* *ikea-big-plate-edge*))
  "in meters")
(defparameter *ikea-big-plate-z-offsets* '(0 0 0.10) "in meters")
(defparameter *ikea-big-plate-2nd-z-offsets* '(0 0 0.20) "in meters")

(man-int:def-object-type-to-gripper-transforms :ikea-big-plate '(:left :right) :front
  :grasp-translation `(,*ikea-big-plate-grasp-xy-offset*
                       0.0
                       ,*ikea-big-plate-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-y-24-deg-grasp-rotation*
  :pregrasp-offsets `(,*ikea-big-plate-pregrasp-xy-offset*
                      0.0
                      ,*ikea-big-plate-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(,*ikea-big-plate-2nd-pregrasp-xy-offset*
                          0.0
                          ,*ikea-big-plate-2nd-pregrasp-z-offset*)
  :lift-translation *ikea-big-plate-z-offsets*
  :2nd-lift-translation *ikea-big-plate-2nd-z-offsets*)

(man-int:def-object-type-to-gripper-transforms :ikea-big-plate '(:left :right) :left-side
  :grasp-translation `(0.0
                       ,*ikea-big-plate-grasp-xy-offset*
                       ,*ikea-big-plate-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-y-24-deg-grasp-rotation*
  :pregrasp-offsets `(0.0
                      ,*ikea-big-plate-pregrasp-xy-offset*
                      ,*ikea-big-plate-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0
                          ,*ikea-big-plate-2nd-pregrasp-xy-offset*
                          ,*ikea-big-plate-pregrasp-z-offset*)
  :lift-translation *ikea-big-plate-z-offsets*
  :2nd-lift-translation *ikea-big-plate-2nd-z-offsets*)

(man-int:def-object-type-to-gripper-transforms :ikea-big-plate '(:left :right) :right-side
  :grasp-translation `(0.0
                       ,(- *ikea-big-plate-grasp-xy-offset*)
                       ,*ikea-big-plate-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-y-24-deg-grasp-rotation*
  :pregrasp-offsets `(0.0
                      ,(- *ikea-big-plate-pregrasp-xy-offset*)
                      ,*ikea-big-plate-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0
                          ,(- *ikea-big-plate-2nd-pregrasp-xy-offset*)
                          ,*ikea-big-plate-2nd-pregrasp-z-offset*)
  :lift-translation *ikea-big-plate-z-offsets*
  :2nd-lift-translation *ikea-big-plate-2nd-z-offsets*)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; POT ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *pot-grasp-x-offset* 0.14 "in meters")
(defparameter *pot-grasp-z-offset* 0.05 "in meters")

;; SIDE grasp
(man-int:def-object-type-to-gripper-transforms :pot '(:left :right) :left-side
  :grasp-translation `(,*pot-grasp-x-offset* 0.0 ,*pot-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-y-grasp-rotation*
  :pregrasp-offsets `(,*plate-pregrasp-y-offset* 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*plate-pregrasp-y-offset* 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :pot '(:left :right) :right-side
  :grasp-translation `(,(- *pot-grasp-x-offset*) 0.0 ,*pot-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-y-flipped-grasp-rotation*
  :pregrasp-offsets `(,(- *plate-pregrasp-y-offset*) 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *plate-pregrasp-y-offset*) 0.0)
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
;; (defparameter *cup-eco-orange-grasp-xy-offset* 0.02 "in meters")
(defparameter *cup-grasp-xy-offset* 0.02 "in meters")
;; (defparameter *cup-eco-orange-grasp-z-offset* 0.01 "in meters")
(defparameter *cup-grasp-z-offset* 0.01 "in meters")
(defparameter *cup-top-grasp-x-offset* 0.05 "in meters")
;; (defparameter *cup-eco-orange-top-grasp-z-offset* 0.02 "in meters")
(defparameter *cup-top-grasp-z-offset* 0.04 "in meters")

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms :cup '(:left :right) :top
  :grasp-translation `(,(- *cup-top-grasp-x-offset*) 0.0d0 ,*cup-top-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :cup '(:left :right) :top2
  :grasp-translation `(0.0d0 ,(- *cup-top-grasp-x-offset*) ,*cup-top-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; BOTTOM grasp
(man-int:def-object-type-to-gripper-transforms :cup '(:left :right) :bottom
  :grasp-translation `(0.0d0 0.0d0 ,(- *cup-grasp-z-offset*))
  :grasp-rot-matrix man-int:*-z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,(- *lift-z-offset*))
  :2nd-pregrasp-offsets `(0.0 0 ,(- *lift-z-offset*))
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; SIDE grasp
(man-int:def-object-type-to-gripper-transforms :cup '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,(- *cup-grasp-xy-offset*) ,*cup-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-flipped-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*cup-pregrasp-xy-offset* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*cup-pregrasp-xy-offset* 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :cup '(:left :right) :right-side
  :grasp-translation `(0.0d0 ,*cup-grasp-xy-offset* ,*cup-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-z-flipped-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *cup-pregrasp-xy-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *cup-pregrasp-xy-offset*) 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)


;; BACK grasp
(man-int:def-object-type-to-gripper-transforms :cup '(:left :right) :back
  :grasp-translation `(,*cup-grasp-xy-offset* 0.0d0 ,*cup-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation-2*
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



;;;;;;;;;;;;;;;;;;;;;;;;;;;;; jeroen-cup ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *jeroen-cup-grasp-xy-offset* 0.02 "in meters")
(defparameter *jeroen-cup-grasp-z-offset* 0.0 "in meters")
(defparameter *jeroen-cup-pregrasp-xy-offset* 0.15 "in meters")
(defparameter *jeroen-cup-pregrasp-z-offset* 0.0 "in meters")
(defparameter *jeroen-cup-top-grasp-x-offset* 0.0"in meters")
(defparameter *jeroen-cup-top-grasp-z-offset* 0.03 "in meters")
(defparameter *jeroen-cup-bottom-pregrasp-z-offset* 0.01 "in meters")
(defparameter *jeroen-cup-bottom-lift-z-offset* 0.01 "in meters")

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms :jeroen-cup '(:left :right) :top
  :grasp-translation `(,(- *jeroen-cup-top-grasp-x-offset*) 0.0d0 ,*jeroen-cup-top-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :jeroen-cup '(:left :right) :top2
  :grasp-translation `(0.0d0 ,(- *jeroen-cup-top-grasp-x-offset*) ,*jeroen-cup-top-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; BOTTOM grasp
(man-int:def-object-type-to-gripper-transforms :jeroen-cup '(:left :right) :bottom
  :grasp-translation `(0.0d0 0.0d0 ,(- *jeroen-cup-grasp-z-offset*))
  :grasp-rot-matrix man-int:*-z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,(- *jeroen-cup-bottom-pregrasp-z-offset*))
  :2nd-pregrasp-offsets `(0.0 0 ,(- *jeroen-cup-bottom-pregrasp-z-offset*))
  :lift-translation `(0 0 ,*jeroen-cup-bottom-lift-z-offset*)
  :2nd-lift-translation `(0 0 ,*jeroen-cup-bottom-lift-z-offset*))

;; SIDE grasp
(man-int:def-object-type-to-gripper-transforms :jeroen-cup '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,(- *jeroen-cup-grasp-xy-offset*) ,*jeroen-cup-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-flipped-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*jeroen-cup-pregrasp-xy-offset* ,*jeroen-cup-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*jeroen-cup-pregrasp-xy-offset* 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :jeroen-cup '(:left :right) :right-side
  :grasp-translation `(0.0d0 ,*jeroen-cup-grasp-xy-offset* ,*jeroen-cup-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-z-flipped-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *jeroen-cup-pregrasp-xy-offset*) ,*jeroen-cup-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *jeroen-cup-pregrasp-xy-offset*) 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; BACK grasp
(man-int:def-object-type-to-gripper-transforms :jeroen-cup '(:left :right) :back
  :grasp-translation `(,*jeroen-cup-grasp-xy-offset* 0.0d0 ,*jeroen-cup-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation-2*
  :pregrasp-offsets `(,(- *jeroen-cup-pregrasp-xy-offset*) 0.0 ,*jeroen-cup-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(,(- *jeroen-cup-pregrasp-xy-offset*) 0.0 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; FRONT grasp
(man-int:def-object-type-to-gripper-transforms :jeroen-cup '(:left :right) :front
  :grasp-translation `(,(- *jeroen-cup-grasp-xy-offset*) 0.0d0 ,*jeroen-cup-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(,*jeroen-cup-pregrasp-xy-offset* 0.0 ,*jeroen-cup-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(,*jeroen-cup-pregrasp-xy-offset* 0.0 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)




;;;;;;;;;;;;;;;;;;;;;;;;;;;;; mug ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *mug-back-grasp-x-offset* 0.04 "in meters")
(defparameter *mug-side-grasp-x-offset* 0.02 "in meters")
(defparameter *mug-top-grasp-y-offset* 0.045 "in meters")

;; FRONT grasp
(man-int:def-object-type-to-gripper-transforms :mug '(:left :right) :front
  :grasp-translation `(,(- *cup-grasp-xy-offset*) 0.0d0 ,*cup-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation-2*
  :pregrasp-offsets `(,*cup-pregrasp-xy-offset* 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,*cup-pregrasp-xy-offset* 0.0 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; BACK grasp -- handle
(man-int:def-object-type-to-gripper-transforms :mug '(:left :right) :back
  :grasp-translation `(,(- *mug-back-grasp-x-offset*) 0.0d0 ,*cup-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation-2*
  :pregrasp-offsets `(,(- *cup-pregrasp-xy-offset*) 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,(- *cup-pregrasp-xy-offset*) 0.0 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; SIDE grasp
(man-int:def-object-type-to-gripper-transforms :mug '(:left :right) :left-side
  :grasp-translation `(,*mug-side-grasp-x-offset* ,(- *cup-grasp-xy-offset*) 0.0)
  :grasp-rot-matrix man-int:*y-across-z-flipped-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*cup-pregrasp-xy-offset* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*cup-pregrasp-xy-offset* 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :mug '(:left :right) :right-side
  :grasp-translation `(,*mug-side-grasp-x-offset* ,*cup-grasp-xy-offset* 0.0)
  :grasp-rot-matrix man-int:*-y-across-z-flipped-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *cup-pregrasp-xy-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *cup-pregrasp-xy-offset*) 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms :mug '(:left :right) :top
  :grasp-translation `(,*mug-side-grasp-x-offset* ,(- *mug-top-grasp-y-offset*) 0.0)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :mug '(:left :right) :top2
  :grasp-translation `(,*mug-side-grasp-x-offset* ,*mug-top-grasp-y-offset* 0.0)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; BOTTOM grasp
(man-int:def-object-type-to-gripper-transforms :mug '(:left :right) :bottom
  :grasp-translation `(,*mug-side-grasp-x-offset* 0.0d0 0.0)
  :grasp-rot-matrix man-int:*-z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,(- *lift-z-offset*))
  :2nd-pregrasp-offsets `(0.0 0 ,(- *lift-z-offset*))
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)




;;;;;;;;;;;;;;;;;;;;;;;;;;;;; shoe ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *shoe-top-grasp-x-offset* 0.1 "in meters")
(defparameter *shoe-top2-grasp-x-offset* 0.05 "in meters")
(defparameter *shoe-front-grasp-x-offset* 0.07 "in meters")
(defparameter *shoe-back-grasp-x-offset* 0.03 "in meters")
(defparameter *shoe-side-grasp-z-offset* 0.03 "in meters")

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms :shoe '(:left :right) :top
  :grasp-translation `(,(- *shoe-top-grasp-x-offset*) 0.0d0 ,*cup-top-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :shoe '(:left :right) :top2
  :grasp-translation `(,(- *shoe-top2-grasp-x-offset*)
                        ,(- *cup-top-grasp-x-offset*)
                        ,*cup-top-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; SIDE grasp
(man-int:def-object-type-to-gripper-transforms :shoe '(:left :right) :left-side
  :grasp-translation `(,(- *shoe-top2-grasp-x-offset*)
                        ,(- *cup-grasp-xy-offset*)
                        ,*shoe-side-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*cup-pregrasp-xy-offset* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*cup-pregrasp-xy-offset* 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :shoe '(:left :right) :right-side
  :grasp-translation `(,(- *shoe-top2-grasp-x-offset*)
                        ,*cup-grasp-xy-offset*
                        ,*shoe-side-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *cup-pregrasp-xy-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *cup-pregrasp-xy-offset*) 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; BACK grasp
(man-int:def-object-type-to-gripper-transforms :shoe '(:left :right) :back
  :grasp-translation `(,(- *shoe-back-grasp-x-offset*) 0.0d0 ,*shoe-side-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation*
  :pregrasp-offsets `(,(- *cup-pregrasp-xy-offset*) 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,(- *cup-pregrasp-xy-offset*) 0.0 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; FRONT grasp
(man-int:def-object-type-to-gripper-transforms :shoe '(:left :right) :front
  :grasp-translation `(,(- *shoe-front-grasp-x-offset*) 0.0d0 ,*shoe-side-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(,*cup-pregrasp-xy-offset* 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,*cup-pregrasp-xy-offset* 0.0 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)





;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; milk ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *milk-grasp-xy-offset* 0.01 "in meters")
(defparameter *milk-grasp-z-offset* 0.03 "in meters")
(defparameter *milk-pregrasp-xy-offset* 0.15 "in meters")
(defparameter *milk-lift-z-offset* 0.15 "in meters")

;; BACK grasp
(man-int:def-object-type-to-gripper-transforms :milk '(:left :right) :back
  :grasp-translation `(,*milk-grasp-xy-offset* 0.0d0 ,*milk-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation*
  :pregrasp-offsets `(,(- *milk-pregrasp-xy-offset*) 0.0 ,*milk-lift-z-offset*)
  :2nd-pregrasp-offsets `(,(- *milk-pregrasp-xy-offset*) 0.0 0.0)
  :lift-translation `(0.0 0.0 ,*milk-lift-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*milk-lift-z-offset*))

;; FRONT grasp
(man-int:def-object-type-to-gripper-transforms :milk '(:left :right) :front
  :grasp-translation `(,(- *milk-grasp-xy-offset*) 0.0d0 ,*milk-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(,*milk-pregrasp-xy-offset* 0.0 ,*milk-lift-z-offset*)
  :2nd-pregrasp-offsets `(,*milk-pregrasp-xy-offset* 0.0 0.0)
  :lift-translation `(0.0 0.0 ,*milk-lift-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*milk-lift-z-offset*))

;; SIDE grasp
(man-int:def-object-type-to-gripper-transforms :milk '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,(- *milk-grasp-xy-offset*) ,*milk-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*milk-pregrasp-xy-offset* ,*milk-lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*milk-pregrasp-xy-offset* 0.0)
  :lift-translation `(0.0 0.0 ,*milk-lift-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*milk-lift-z-offset*))

(man-int:def-object-type-to-gripper-transforms :milk '(:left :right) :right-side
  :grasp-translation `(0.0d0 ,*milk-grasp-xy-offset* ,*milk-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *milk-pregrasp-xy-offset*) ,*milk-lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *milk-pregrasp-xy-offset*) 0.0)
  :lift-translation `(0.0 0.0 ,*milk-lift-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*milk-lift-z-offset*))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; cereal ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *cereal-grasp-z-offset* 0.04 "in meters")
(defparameter *cereal-grasp-side-z-offset* 0.0 "in meters")
(defparameter *cereal-grasp-xy-offset* 0.015 "in meters")
(defparameter *cereal-pregrasp-z-offset* 0.05 "in meters")
(defparameter *cereal-pregrasp-xy-offset* 0.15 "in meters")
(defparameter *cereal-postgrasp-xy-offset* 0.40 "in meters")
(defparameter *cereal-lift-z-offset* 0.1 "in meters")
(defparameter *cereal-small-lift-z-offset* 0.06 "in meters")

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms
    '(:cereal :breakfast-cereal) '(:left :right) :top
  :grasp-translation `(0.0d0 0.0d0 ,*cereal-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; FRONT grasp table
(man-int:def-object-type-to-gripper-transforms
    '(:cereal :breakfast-cereal) '(:left :right) :front
  :grasp-translation `(,*cereal-grasp-xy-offset* 0.0d0 ,*cereal-grasp-side-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(,*cereal-pregrasp-xy-offset* 0.0 ,*cereal-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(,*cereal-pregrasp-xy-offset* 0.0 0.0)
  :lift-translation `(0.0 0.0 ,*cereal-lift-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*cereal-lift-z-offset*))

;; FRONT grasp table flipped because Donbot has an eye on his hand
(man-int:def-object-type-to-gripper-transforms
    '(:cereal :breakfast-cereal) '(:left :right) :front-flipped
  :grasp-translation `(,*cereal-grasp-xy-offset* 0.0d0 ,*cereal-grasp-side-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation-2*
  :pregrasp-offsets `(,*cereal-pregrasp-xy-offset* 0.0 ,*cereal-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(,*cereal-pregrasp-xy-offset* 0.0 0.0)
  :lift-translation `(0.0 0.0 ,*cereal-lift-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*cereal-lift-z-offset*))

;; FRONT grasp shelf
(man-int:def-object-type-to-gripper-transforms
    '(:cereal :breakfast-cereal) '(:left :right) :front
  :location-type :shelf
  :grasp-translation `(,*cereal-grasp-xy-offset* 0.0d0 ,*cereal-grasp-side-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(,*cereal-pregrasp-xy-offset* 0.0 ,*cereal-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(,*cereal-pregrasp-xy-offset* 0.0 0.0)
  :lift-translation `(,*cereal-grasp-xy-offset* 0.0 ,*cereal-small-lift-z-offset*)
  :2nd-lift-translation `(,*cereal-postgrasp-xy-offset* 0.0 ,*cereal-small-lift-z-offset*))

;; BACK grasp table
(man-int:def-object-type-to-gripper-transforms
    '(:cereal :breakfast-cereal) '(:left :right) :back
  :grasp-translation `(,(- *cereal-grasp-xy-offset*) 0.0d0 ,*cereal-grasp-side-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation*
  :pregrasp-offsets `(,(- *cereal-pregrasp-xy-offset*) 0.0 ,*cereal-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(,(- *cereal-pregrasp-xy-offset*) 0.0 0.0)
  :lift-translation `(0.0 0.0 ,*cereal-lift-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*cereal-lift-z-offset*))

;; BACK grasp table flipped because Donbot has an eye on his hand
(man-int:def-object-type-to-gripper-transforms
    '(:cereal :breakfast-cereal) '(:left :right) :back-flipped
  :grasp-translation `(,(- *cereal-grasp-xy-offset*) 0.0d0 ,*cereal-grasp-side-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation-2*
  :pregrasp-offsets `(,(- *cereal-pregrasp-xy-offset*) 0.0 ,*cereal-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(,(- *cereal-pregrasp-xy-offset*) 0.0 0.0)
  :lift-translation `(0.0 0.0 ,*cereal-lift-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*cereal-lift-z-offset*))

;; BACK grasp shelf
(man-int:def-object-type-to-gripper-transforms
    '(:cereal :breakfast-cereal) '(:left :right) :back
  :location-type :shelf
  :grasp-translation `(,(- *cereal-grasp-xy-offset*) 0.0d0 ,*cereal-grasp-side-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation*
  :pregrasp-offsets `(,(- *cereal-pregrasp-xy-offset*) 0.0 ,*cereal-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(,(- *cereal-pregrasp-xy-offset*) 0.0 0.0)
  :lift-translation `(,(- *cereal-grasp-xy-offset*) 0.0 ,*cereal-small-lift-z-offset*)
  :2nd-lift-translation `(,(- *cereal-postgrasp-xy-offset*) 0.0 ,*cereal-small-lift-z-offset*))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;; bowl ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; (defparameter *edeka-red-bowl-grasp-x-offset* 0.07 "in meters")
(defparameter *bowl-grasp-x-offset* 0.08 "in meters")
(defparameter *bowl-tilted-pregrasp-x-offset* 0.08 "in meters")
;; (defparameter *edeka-red-bowl-grasp-z-offset* 0.0 "in meters")
(defparameter *bowl-grasp-z-offset* 0.02 "in meters")
(defparameter *bowl-tilted-grasp-z-offset* 0.04 "in meters")
(defparameter *bowl-pregrasp-z-offset* 0.20 "in meters")

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms :bowl '(:left :right) :top
  :grasp-translation `(,(- *bowl-grasp-x-offset*) 0.0d0 ,*bowl-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :bowl '(:left :right) :top-front
  :grasp-translation `(,*bowl-grasp-x-offset* 0.0d0 ,*bowl-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :bowl '(:left :right) :top-left
  :grasp-translation `(0.0d0 ,*bowl-grasp-x-offset* ,*bowl-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)
(man-int:def-object-type-to-gripper-transforms :bowl '(:left :right) :top-left-tilted
  :grasp-translation `(0.0d0 ,*bowl-grasp-x-offset* ,*bowl-tilted-grasp-z-offset*)
  :grasp-rot-matrix
  `((0                                    1 0)
    (,(sin *plate-grasp-roll-offset*)     0 ,(- (cos *plate-grasp-roll-offset*)))
    (,(- (cos *plate-grasp-roll-offset*)) 0 ,(- (sin *plate-grasp-roll-offset*))))
  :pregrasp-offsets `(0.0d0 ,*bowl-tilted-pregrasp-x-offset* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0d0 ,*bowl-tilted-pregrasp-x-offset* ,*lift-z-offset*)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :bowl '(:left :right) :top-right
  :grasp-translation `(0.0d0 ,(- *bowl-grasp-x-offset*) ,*bowl-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)
(man-int:def-object-type-to-gripper-transforms :bowl '(:left :right) :top-right-tilted
  :grasp-translation `(0.0d0 ,(- *bowl-grasp-x-offset*) ,*bowl-tilted-grasp-z-offset*)
  :grasp-rot-matrix
  `((0 -1 0)
    (,(- (sin *plate-grasp-roll-offset*)) 0 ,(cos *plate-grasp-roll-offset*))
    (,(- (cos *plate-grasp-roll-offset*)) 0 ,(- (sin *plate-grasp-roll-offset*))))
  :pregrasp-offsets `(0.0d0 ,(- *bowl-tilted-pregrasp-x-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0d0 ,(- *bowl-tilted-pregrasp-x-offset*) ,*lift-z-offset*)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; TOP grasp drawer
(man-int:def-object-type-to-gripper-transforms :bowl '(:left :right) :top
  :location-type :drawer
  :grasp-translation `(,(- *bowl-grasp-x-offset*) 0.0d0 ,*bowl-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*bowl-pregrasp-z-offset*))
(man-int:def-object-type-to-gripper-transforms :bowl '(:left :right) :top-front
  :location-type :drawer
  :grasp-translation `(,*bowl-grasp-x-offset* 0.0d0 ,*bowl-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*bowl-pregrasp-z-offset*))
(man-int:def-object-type-to-gripper-transforms :bowl '(:left :right) :top-left
  :location-type :drawer
  :grasp-translation `(0.0d0 ,*bowl-grasp-x-offset* ,*bowl-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*bowl-pregrasp-z-offset*))
(man-int:def-object-type-to-gripper-transforms :bowl '(:left :right) :top-right
  :location-type :drawer
  :grasp-translation `(0.0d0 ,(- *bowl-grasp-x-offset*) ,*bowl-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*bowl-pregrasp-z-offset*))

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
  :2nd-lift-translation `(0.0 0.0 ,*weisswurst-pregrasp-z-offset*))

;; Left-TOP grasp
(man-int:def-object-type-to-gripper-transforms '(:weisswurst)
    '(:left :right) :left-top
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*)

;; Right-TOP grasp
(man-int:def-object-type-to-gripper-transforms '(:weisswurst)
    '(:left :right) :right-top
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*)

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


;;;;;;;;;;;;;;;;;;;;;;;;;;;;; BREAD;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defparameter *bread-grasp-z-offset* 0.0 ;; 0.015
              "in meters") ; because TCP is not at the edge
(defparameter *bread-pregrasp-z-offset* 0.18 "in meters")
(defparameter *bread-pregrasp-xy-offset* 0.10 "in meters")


;; Left-TOP grasp
(man-int:def-object-type-to-gripper-transforms '(:bread)
    '(:left :right) :left-top
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*)

;; right-TOP grasp
(man-int:def-object-type-to-gripper-transforms '(:bread)
    '(:left :right) :right-top
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*)

;; left hold
(man-int:def-object-type-to-gripper-transforms '(:bread)
    '(:left :right) :left-hold
  :grasp-translation `(0.1 0.0 ,*bread-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(0.1 0.0 ,*bread-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.1 0.0 ,*bread-pregrasp-z-offset*)
  :lift-translation `(0.1 0.0 ,*bread-pregrasp-z-offset*)
  :2nd-lift-translation `(0.1 0.0 ,*bread-pregrasp-z-offset*))

;; right-hold
(man-int:def-object-type-to-gripper-transforms '(:bread)
    '(:left :right) :right-hold
  :grasp-translation `(-0.1 0.0 ,*bread-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation*
  :pregrasp-offsets `(-0.1 0.0 ,*bread-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(-0.1 0.0 ,*bread-pregrasp-z-offset*)
  :lift-translation `(-0.1 0.0 ,*bread-pregrasp-z-offset*)
  :2nd-lift-translation `(-0.1 0.0 ,*bread-pregrasp-z-offset*))


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
  :lift-translation `(0.0 0.06 ,*cutlery-pregrasp-z-offset*)
  :2nd-lift-translation `(0.0 0.06 ,*cutlery-pregrasp-z-offset*))

;;;;;;;;;;;;;;;;;;;;;;; POPCORN-POT ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; TOP GRASP
(man-int:def-object-type-to-gripper-transforms :popcorn-pot '(:left :right) :top
  :grasp-translation `(0.0 0.105 0.025)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 0.08)
  :2nd-pregrasp-offsets `(0.0 0.0 0.08)
  :lift-translation `(0.0 0.0 0.02)
  :2nd-lift-translation `(0.0 0.0 0.02))

(man-int:def-object-type-to-gripper-transforms :popcorn-pot '(:left :right) :left-side
  :grasp-translation `(0.1315 0.0114 0.031)
  :grasp-rot-matrix man-int:*x-across-y-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 0.08)
  :2nd-pregrasp-offsets `(0.0 0.0 0.08)
  :lift-translation `(0.0 0.0 0.12)
  :2nd-lift-translation `(0.0 0.0 0.12))


(man-int:def-object-type-to-gripper-transforms :popcorn-pot '(:left :right) :right-side
  :grasp-translation `(-0.128 -0.004 0.0309)
  :grasp-rot-matrix man-int::*-x-across-y-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 0.08)
  :2nd-pregrasp-offsets `(0.0 0.0 0.08)
  :lift-translation `(0.0 0.0 0.12)
  :2nd-lift-translation `(0.0 0.0 0.12))


;;;;;;;;;;;;;;;;;;;;;; POPCORN-POT-LID ;;;;;;;;;;;;;;;;;;;;;;;;;;

;; TOP GRASP
(man-int:def-object-type-to-gripper-transforms :popcorn-pot-lid '(:left :right) :top
  :grasp-translation `(0.0 0.0 0.01)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation  `(0.0 0.0 0.01)
  :2nd-lift-translation  `(0.0 0.0 0.01))

;;;;;;;;;;;;;;;;;;;;;;; IKEA-BOWL-WW ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; TOP GRASP
(man-int:def-object-type-to-gripper-transforms :ikea-bowl-ww '(:left :right) :top
  :grasp-translation `(0.0 0.075 0.025)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;;;;;;;;;;;;;;;;;;;;;;; SALT ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; TOP GRASP
(man-int:def-object-type-to-gripper-transforms :salt '(:left) :left-side
  :grasp-translation `(0.0 0.0 0.0)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :salt '(:right) :right-side
  :grasp-translation `(0.0 0.0 0.0)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;;;;;;;;;;;;;;;;;;;;;;; IKEA-PLATE ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;; TOP GRASP
(man-int:def-object-type-to-gripper-transforms :ikea-plate '(:left :right) :top
  :grasp-translation `(-0.14 0.0 0.025)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :ikea-plate '(:right) :front
  :grasp-translation `(0.0 0.0 0.0)
  :grasp-rot-matrix man-int:*-y-across-x-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :ikea-plate '(:left) :front
  :grasp-translation `(0.0 0.0 0.0)
  :grasp-rot-matrix man-int:*y-across-x-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(defmethod man-int:get-object-type-robot-frame-tilt-approach-transform
    ((object-type (eql :ikea-plate))
     (arm (eql :left))
     (grasp (eql :left-side)))
  '((-0.02 0.02 0.18)(0 0 0 1)))

(defmethod man-int:get-object-type-robot-frame-tilt-approach-transform
    ((object-type (eql :ikea-plate))
     (arm (eql :right))
     (grasp (eql :right-side)))
  '((-0.02 0.02 0.14)(0 0 0 1)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defmethod man-int:get-action-gripping-effort :heuristics 20 ((object-type (eql :popcorn-item)))
  35)

(defmethod man-int:get-action-gripper-opening :heuristics 20 ((object-type (eql :popcorn-item)))
  0.1)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(man-int:def-object-type-in-other-object-transform :popcorn-pot-lid :popcorn-pot :popcorn-pot-lid-attachment
  :attachment-translation `(0.0 0.0 0.0745)
  :attachment-rot-matrix man-int:*identity-matrix*)

(defmethod man-int:get-z-offset-for-placing-with-dropping
    ((other-object (eql :popcorn-pot))
     (object (eql :popcorn-pot-lid))
     (attachment (eql :popcorn-pot-lid-attachment)))
  0.02)

(def-fact-group popcorn-object-type-hierarchy (man-int:object-type-direct-subtype)
  (<- (man-int:object-type-direct-subtype :popcorn-item :popcorn-pot))
  (<- (man-int:object-type-direct-subtype :popcorn-item :popcorn-pot-lid))
  (<- (man-int:object-type-direct-subtype :popcorn-item :ikea-bowl-ww))
  (<- (man-int:object-type-direct-subtype :popcorn-item :ikea-plate))
  (<- (man-int:object-type-direct-subtype :popcorn-item :salt)))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;; bowl-round ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *bowl-round-grasp-x-offset* 0.065 "in meters")
(defparameter *bowl-round-grasp-z-offset* 0.02 "in meters")
(defparameter *bowl-round-lift-z-offset* 0.04 "in meters")

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms :bowl-round '(:left :right) :top
  :grasp-translation `(,(- *bowl-round-grasp-x-offset*) 0.0d0 ,*bowl-round-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*bowl-round-lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*bowl-round-lift-z-offset*)
  :lift-translation `(0.0 0.0 ,*bowl-round-lift-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*bowl-round-lift-z-offset*))

(man-int:def-object-type-to-gripper-transforms :bowl-round '(:left :right) :top-front
  :grasp-translation `(,*bowl-round-grasp-x-offset* 0.0d0 ,*bowl-round-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*bowl-round-lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*bowl-round-lift-z-offset*)
  :lift-translation `(0.0 0.0 ,*bowl-round-lift-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*bowl-round-lift-z-offset*))

(man-int:def-object-type-to-gripper-transforms :bowl-round '(:left :right) :top-left
  :grasp-translation `(0.0d0 ,*bowl-round-grasp-x-offset* ,*bowl-round-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*bowl-round-lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*bowl-round-lift-z-offset*)
  :lift-translation `(0.0 0.0 ,*bowl-round-lift-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*bowl-round-lift-z-offset*))

(man-int:def-object-type-to-gripper-transforms :bowl-round '(:left :right) :top-right
  :grasp-translation `(0.0d0 ,(- *bowl-round-grasp-x-offset*) ,*bowl-round-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*bowl-round-lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*bowl-round-lift-z-offset*)
  :lift-translation `(0.0 0.0 ,*bowl-round-lift-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*bowl-round-lift-z-offset*))




;;;;;;;;;;;;;;;;;;;;;;;;; table setting locations ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;; utilities

;;;;;;;; sink

(defun make-location-on-sink-left (?environment-name)
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name sink-area-surface)
                         (owl-name "kitchen_sink_block_counter_top")
                         (part-of ?environment-name)))
           ;; left in the room is right in the sink-area-surface frame
           (side right)))

(defun make-location-on-sink-left-front (?environment-name)
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name sink-area-surface)
                         (owl-name "kitchen_sink_block_counter_top")
                         (part-of ?environment-name)))
           ;; left-front in map frame is right back in sink-area-surface frame
           (side (right back))
           (range-invert 0.5)))

(defun make-location-on-sink-middle-front (?environment-name)
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name sink-area-surface)
                         (owl-name "kitchen_sink_block_counter_top")
                         (part-of ?environment-name)))
           (side (right back))
           (range 0.5)))

(defun make-location-in-sink (?object-type ?environment-name)
  (desig:a location
           (above (desig:an object
                            (type sink)
                            (urdf-name sink-area-sink)
                            (part-of ?environment-name)))
           (side left)
           (for (desig:an object (type ?object-type)))
           ;; the "for" condition for spoon adds a height that is too high to reach
           ;; so adding a little negative z-offset
           (z-offset -0.1)))

;;;;;;;;; sink drawers

(defun make-location-in-sink-left-upper-drawer (?environment-name)
  (desig:a location
           (in (desig:an object
                         (type drawer)
                         (urdf-name sink-area-left-upper-drawer-main)
                         (owl-name "drawer_sinkblock_upper_open")
                         (part-of ?environment-name)))
           (side back)))

(defun make-location-in-sink-left-bottom-drawer (?environment-name)
  (desig:a location
           (in (desig:an object
                         (type drawer)
                         (urdf-name sink-area-left-bottom-drawer-main)
                         (part-of ?environment-name)))
           (side back)))

(defun make-location-in-sink-left-middle-drawer (?environment-name)
  (desig:a location
           (in (desig:an object
                         (type drawer)
                         (urdf-name sink-area-left-middle-drawer-main)
                         (owl-name "drawer_sinkblock_middle_open")
                         (part-of ?environment-name)))
           (side back)))

(defun make-location-in-sink-trash-drawer (?object-type ?environment-name)
  (desig:a location
           (above (desig:an object
                            (type drawer)
                            (urdf-name sink-area-trash-drawer-main)
                            (part-of ?environment-name)))
           (z-offset -0.05)
           (side (back left))
           (range 0.2)
           (for (desig:an object (type ?object-type)))))

;;;;;;;; dishwasher

(defun make-location-in-dishwasher-drawer (?object-type ?environment-name)
  (let ((?location-in-dishwasher
          (desig:a location
                   (in (desig:an object
                                 (type dishwasher)
                                 (urdf-name sink-area-dish-washer-main)
                                 (part-of ?environment-name)))))
        (?attachments
          (case ?object-type
            (:bowl
             '(;; :dish-washer-drawer-left
               :dish-washer-drawer-right))
            (:cup
             '(:dish-washer-drawer-left-flipped-around-x
               :dish-washer-drawer-left-flipped-around-y))
            (:spoon
             '(:dish-washer-drawer-center)))))
    (desig:a location
             (above (desig:an object
                              (type drawer)
                              (urdf-name sink-area-dish-washer-tray-bottom)
                              (part-of ?environment-name)
                              (location ?location-in-dishwasher)))
             (for (desig:an object
                            (type ?object-type)
                            ;; need a name because of the attachment
                            (name some-name)))
             (attachments ?attachments))))

;;;;;;;; vertical drawer

(defun make-location-in-oven-right-drawer (?object-type ?environment-name)
  (desig:a location
           (in (desig:an object
                         (type drawer)
                         (urdf-name oven-area-area-right-drawer-main)
                         (owl-name "drawer_oven_right_open")
                         (part-of ?environment-name)
                         (level topmost)))
           (side back)
           (range 0.1)
           (orientation support-aligned)
           (for (desig:an object (type ?object-type)))))

;;;;;;;; fridge

(defun make-location-in-fridge (?environment-name)
  (desig:a location
           (in (desig:an object
                         (type fridge)
                         (urdf-name iai-fridge-main)
                         (owl-name "drawer_fridge_upper_interior")
                         (part-of ?environment-name)
                         (level topmost)))))

(defun make-location-in-fridge-door (?environment-name)
  (desig:a location
           (in (desig:an object
                         (type fridge)
                         (urdf-name iai-fridge-door)
                         (owl-name "drawer_fridge_door_open")
                         (part-of ?environment-name)
                         (level bottommost)))))

;;;;;;;; kitchen island

(defun make-location-on-kitchen-island-slots (?object-type ?environment-name)
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name kitchen-island-surface)
                         (owl-name "kitchen_island_counter_top")
                         (part-of ?environment-name)))
           (for (desig:an object (type ?object-type)))
           (side (back right))
           (range-invert 0.5)
           (context table-setting)
           (object-count 3)))

(defun make-location-on-kitchen-island (?object-type ?environment-name)
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name kitchen-island-surface)
                         (owl-name "kitchen_island_counter_top")
                         (part-of ?environment-name)))
           (for (desig:an object (type ?object-type)))
           (side (back right))))

(defun make-cereal-location (?object-type ?environment-name)
  (desig:a location
           ;; (left-of (desig:an object (type ?other-object)))
           ;; (far-from (desig:an object (type ?other-object)))
           ;; (orientation axis-aligned)
           (on (desig:an object
                         (type counter-top)
                         (urdf-name kitchen-island-surface)
                         (owl-name "kitchen_island_counter_top")
                         (part-of ?environment-name)))
           (for (desig:an object (type ?object-type)))
           (side (back right))))

(defun make-location-in-kitchen-island-left-upper-drawer (?environment-name)
  (desig:a location
           (in (desig:an object
                         (type drawer)
                         (urdf-name kitchen-island-left-upper-drawer-main)
                         (part-of ?environment-name)))
           (side back)))

;;;;;;;; dining table

(defun make-location-on-dining-table-slots (?object-type ?environment-name)
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name dining-area-jokkmokk-table-main)
                         (part-of ?environment-name)))
           (for (desig:an object (type ?object-type)))
           (side front)
           (context table-setting)
           (object-count 2)))

(defun make-location-on-dining-table (?environment-name)
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name dining-area-jokkmokk-table-main)
                         (part-of ?environment-name)))
           (side (front right))))

(defun make-location-in-center-of-dining-table (?object-type ?environment-name)
  (desig:a location
           (on (desig:an object
                         (type counter-top)
                         (urdf-name dining-area-jokkmokk-table-main)
                         (part-of ?environment-name)))
           (for (desig:an object (type ?object-type)))
           (range 0.2)
           (side front)))

;;;;;;;;;; w.r.t. other object

(defun make-location-right-of-other-object (?object-type ?other-object-type
                                            ?other-object-location
                                            ;; ?environment-name
                                            )
  (let ((?other-object-designator
          (desig:an object
                    (type ?other-object-type)
                    (location ?other-object-location))))
    (desig:a location
             (right-of ?other-object-designator)
             (threshold 0.9)
             (near ?other-object-designator)
             (for (desig:an object (type ?object-type)))
             ;; Don't add the ON property, otherwise the ORIENTATION will mess up
             ;; (on (desig:an object
             ;;               (type counter-top)
             ;;               (urdf-name dining-area-jokkmokk-table-main)
             ;;               (part-of ?environment-name)))
             (orientation support-aligned))))

(defun make-location-right-of-behind-other-object (?object-type ?other-object-type
                                                   ?other-object-location)
  (let ((?other-object-designator
          (desig:an object
                    (type ?other-object-type)
                    (location ?other-object-location))))
    (desig:a location
             (right-of ?other-object-designator)
             (behind ?other-object-designator)
             (near ?other-object-designator)
             (for (desig:an object (type ?object-type))))))

(defun make-location-left-of-far-other-object (?object-type ?other-object-type
                                               ?other-object-location)
  (let ((?other-object-designator
          (desig:an object
                    (type ?other-object-type)
                    (location ?other-object-location))))
    (desig:a location
             (left-of ?other-object-designator)
             (far-from ?other-object-designator)
             (for (desig:an object (type ?object-type))))))


;;;;;;;;;;;;;;;;;;;; context TABLE-SETTING-COUNTER

;;;;;;;; fetching locations

(mapcar (lambda (type)
          (defmethod man-int:get-object-likely-location :heuristics 20
              ((object-type (eql type))
               environment human
               (context (eql :table-setting-counter)))
            (make-location-on-sink-left-front environment)))
        '(:plate))

(mapcar (lambda (type)
          (defmethod man-int:get-object-likely-location :heuristics 20
              ((object-type (eql type))
               environment human
               (context (eql :table-setting-counter)))
            (make-location-on-sink-middle-front environment)))
        '(:bottle :milk :cereal :breakfast-cereal :cup :bowl :mug))

(mapcar (lambda (type)
          (defmethod man-int:get-object-likely-location :heuristics 20
              ((object-type (eql type))
               environment human
               (context (eql :table-setting-counter)))
            (make-location-in-sink-left-upper-drawer environment)))
        '(:cutlery :spoon))

;;;;;;;;; destination locations

(mapcar (lambda (object-type)
          (defmethod man-int:get-object-destination :heuristics 20
              ((object-type (eql object-type))
               environment human
               (context (eql :table-setting-counter)))
            (make-location-on-kitchen-island-slots object-type environment)))
        '(:bowl :plate))

(mapcar (lambda (object-type)
          (defmethod man-int:get-object-destination :heuristics 20
              ((object-type (eql object-type))
               environment human
               (context (eql :table-setting-counter)))
            (make-location-on-kitchen-island object-type environment)))
        '(:mug :bottle))

(mapcar (lambda (object-type-and-other-object-type)
          (destructuring-bind (object-type other-object-type)
              object-type-and-other-object-type
            (defmethod man-int:get-object-destination :heuristics 20
                ((object-type (eql object-type))
                 environment human
                 (context (eql :table-setting-counter)))
              (make-location-right-of-other-object
               object-type other-object-type
               (make-location-on-kitchen-island-slots
                other-object-type environment)))))
        '((:spoon :bowl)
          (:knife :plate)))

(mapcar (lambda (object-type-and-other-object-type)
          (destructuring-bind (object-type other-object-type)
              object-type-and-other-object-type
            (defmethod man-int:get-object-destination :heuristics 20
                ((object-type (eql object-type))
                 environment human
                 (context (eql :table-setting-counter)))
              (make-location-right-of-behind-other-object
               object-type other-object-type
               (make-location-on-kitchen-island-slots
                other-object-type environment)))))
        '((:cup :bowl)))

(mapcar (lambda (object-type)
          (defmethod man-int:get-object-destination :heuristics 20
              ((object-type (eql object-type))
               environment human
               (context (eql :table-setting-counter)))
            (make-cereal-location object-type environment)))
        '(:cereal :breakfast-cereal))

(defmethod man-int:get-object-destination :heuristics 20
    ((object-type (eql :milk))
     environment human
     (context (eql :table-setting-counter)))
  (make-location-left-of-far-other-object
   object-type :bowl
   (make-location-on-kitchen-island-slots :bowl environment)))

;;;;;;;;;;;;;;;;;;;; context TABLE-SETTING

;;;;;;;;;; fetch locations

(mapcar (lambda (type)
          (defmethod man-int:get-object-likely-location :heuristics 20
              ((object-type (eql type))
               environment human
               (context (eql :table-setting)))
            (make-location-in-sink-left-upper-drawer environment)))
        '(:cutlery :spoon))

(mapcar (lambda (type)
          (defmethod man-int:get-object-likely-location :heuristics 20
              ((object-type (eql type))
               environment human
               (context (eql :table-setting)))
            (make-location-in-sink-left-middle-drawer environment)))
        '(:bowl))

(mapcar (lambda (type)
          (defmethod man-int:get-object-likely-location :heuristics 20
              ((object-type (eql type))
               environment human
               (context (eql :table-setting)))
            ;; (make-location-in-sink-left-bottom-drawer environment)
            (make-location-in-kitchen-island-left-upper-drawer environment)))
        '(:cup))

(mapcar (lambda (type)
          (defmethod man-int:get-object-likely-location :heuristics 20
              ((object-type (eql type))
               environment human
               (context (eql :table-setting)))
            (make-location-in-fridge-door environment)))
        '(:milk))

(mapcar (lambda (type)
          (defmethod man-int:get-object-likely-location :heuristics 20
              ((object-type (eql type))
               environment human
               (context (eql :table-setting)))
            (make-location-in-oven-right-drawer object-type environment)))
        '(:cereal :breakfast-cereal))

;;;;;;;; destination locations

(mapcar (lambda (object-type)
          (defmethod man-int:get-object-destination :heuristics 20
              ((object-type (eql object-type))
               environment human
               (context (eql :table-setting)))
            (make-location-on-dining-table-slots object-type environment)))
        '(:bowl))

(mapcar (lambda (object-type-and-other-object-type)
          (destructuring-bind (object-type other-object-type)
              object-type-and-other-object-type
            (defmethod man-int:get-object-destination :heuristics 20
                ((object-type (eql object-type))
                 environment human
                 (context (eql :table-setting)))
              (make-location-right-of-other-object
               object-type other-object-type
               (make-location-on-dining-table-slots
                other-object-type environment)))))
        '((:spoon :bowl)))

(mapcar (lambda (object-type-and-other-object-type)
          (destructuring-bind (object-type other-object-type)
              object-type-and-other-object-type
            (defmethod man-int:get-object-destination :heuristics 20
                ((object-type (eql object-type))
                 environment human
                 (context (eql :table-setting)))
              (make-location-right-of-behind-other-object
               object-type other-object-type
               (make-location-on-dining-table-slots
                other-object-type environment)))))
        '((:cup :bowl)))

(mapcar (lambda (object-type)
          (defmethod man-int:get-object-destination :heuristics 20
              ((object-type (eql object-type))
               environment human
               (context (eql :table-setting)))
            (make-location-in-center-of-dining-table object-type environment)))
        '(:cereal :breakfast-cereal :milk))

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
            (make-location-on-dining-table environment)))
        '(:bowl :spoon :cup :breakfast-cereal :milk
          :cereal :mug :cutlery :plate :bottle))

(mapcar (lambda (type)
          (defmethod man-int:get-object-destination :heuristics 20
              ((object-type (eql type))
               environment human
               (context (eql :table-cleaning)))
            (make-location-in-oven-right-drawer object-type environment)))
        '(:cereal :breakfast-cereal))

(mapcar (lambda (type)
          (defmethod man-int:get-object-destination :heuristics 20
              ((object-type (eql type))
               environment human
               (context (eql :table-cleaning)))
            (make-location-in-sink object-type environment)))
        '(:plate :mug :cutlery))

(mapcar (lambda (type)
          (defmethod man-int:get-object-destination :heuristics 20
              ((object-type (eql type))
               environment human
               (context (eql :table-cleaning)))
            (make-location-in-dishwasher-drawer object-type environment)))
        '(:bowl :cup :spoon))


(mapcar (lambda (type)
          (defmethod man-int:get-object-destination :heuristics 20
              ((object-type (eql type))
               environment human
               (context (eql :table-cleaning)))
            (make-location-in-sink-trash-drawer object-type environment)))
        '(:milk :bottle))

;; (mapcar (lambda (type)
;;           (defmethod man-int:get-object-destination :heuristics 20
;;               ((object-type (eql type))
;;                environment human
;;                (context (eql :table-cleaning)))
;;             (make-location-on-sink environment object-type)))
;;         '(:bowl :plate :cutlery :mug :cup :cereal :breakfast-cereal :milk :bottle))


;;;;;;;;;;;;;;;;;; Predefined poses for placing on dish-washer-drawer ;;;;;;;;;;;;;

(man-int:def-object-type-in-other-object-transform :bowl :drawer
  :dish-washer-drawer-left
  :attachment-translation `(0.05 -0.15 0.2)
  :attachment-rot-matrix man-int:*identity-matrix*)

(man-int:def-object-type-in-other-object-transform :bowl :drawer
  :dish-washer-drawer-right
  :attachment-translation `(0.03 0.15 0.2)
  :attachment-rot-matrix man-int:*identity-matrix*)

(man-int:def-object-type-in-other-object-transform :cup :drawer
  :dish-washer-drawer-left-flipped-around-x
  :attachment-translation `(0.06 -0.19 0.22)
  :attachment-rot-matrix man-int:*rotation-around-x-180-matrix*)

(man-int:def-object-type-in-other-object-transform :cup :drawer
  :dish-washer-drawer-left-flipped-around-y
  :attachment-translation `(0.06 -0.19 0.22)
  :attachment-rot-matrix man-int:*rotation-around-y-180-matrix*)

(man-int:def-object-type-in-other-object-transform :spoon :drawer
  :dish-washer-drawer-center
  :attachment-translation `(0.04 0.0 0.03)
  :attachment-rot-matrix man-int:*identity-matrix*)

;;;;;;;;;;;;;;;;;; Predefined poses for apartment kitchen ;;;;;;;;;;;;;;;;;;;;;;

(man-int:def-object-type-in-other-object-transform :jeroen-cup :drawer
  :jeroen-cup-in-dishwasher-1
  :attachment-translation `(-0.2 0.15 0.13)
  :attachment-rot-matrix man-int:*rotation-around-x-180-matrix*)

(man-int:def-object-type-in-other-object-transform :jeroen-cup :drawer
  :jeroen-cup-in-dishwasher-2
  :attachment-translation `(-0.2 0.15 0.13)
  :attachment-rot-matrix man-int:*rotation-around-y-180-matrix*)
