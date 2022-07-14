;;;
;;; Copyright (c) 2019, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;               2019, Jonas Dech <jdech[at]uni-bremen.de>
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


(defun make-arm-transform (object-name arm x y z &optional rot-matrix)
  (cl-transforms-stamped:make-transform-stamped
   (roslisp-utilities:rosify-underscores-lisp-name object-name)
   (ecase arm
     (:left cram-tf:*robot-left-tool-frame*)
     (:right cram-tf:*robot-right-tool-frame*))
   0.0
   (cl-transforms:make-3d-vector x y z)
   (if rot-matrix
       (cl-transforms:matrix->quaternion
        (make-array '(3 3) :initial-contents rot-matrix))
       (cl-transforms:make-identity-rotation))))

(defun make-base-transform (x y z)
  (cl-transforms-stamped:make-transform-stamped
    cram-tf:*robot-base-frame*
    cram-tf:*robot-base-frame*
    0.0
    (cl-transforms:make-3d-vector x y z)
    (cl-transforms:make-identity-rotation)))


(defparameter *default-retail-z-offset* 0.05 "in meters")
(defparameter *default-retail-lift-offsets* `(0.0 0.0 ,*default-retail-z-offset*))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-fact-group retail-object-type-hierarchy (man-int:object-type-direct-subtype)
  (<- (man-int:object-type-direct-subtype :retail-item ?item-type)
    (member ?item-type (:dish-washer-tabs
                        :retail-bottle :deodorant :juice-box
                        :denkmit :dove :heitmann :somat :basket)))
  (<- (man-int:object-type-direct-subtype :retail-bottle ?item-type)
    (member ?item-type (:retail-bottle-round :retail-bottle-flat)))
  (<- (man-int:object-type-direct-subtype :retail-bottle-round ?item-type)
    (member ?item-type (:heitmann-citronensaeure :domestos-allzweckreiniger)))
  (<- (man-int:object-type-direct-subtype :retail-bottle-flat ?item-type)
    (member ?item-type (:balea-bottle :denkmit-entkalker :kuehne-essig-essenz))))

(def-fact-group attachment-knowledge (man-int:unidirectional-attachment)
  (<- (man-int:unidirectional-attachment ?attachment-type)
    (member ?attachment-type (:in-basket-front :in-basket-back :in-basket))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-action-gripper-opening
    :heuristics 20 ((object-type (eql :retail-item)))
  0.1)
(defmethod man-int:get-action-gripper-opening
    :heuristics 20 ((object-type (eql :balea-bottle)))
  0.06)
(defmethod man-int:get-action-gripper-opening
    :heuristics 20 ((object-type (eql :dish-washer-tabs)))
  0.06)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-action-gripping-effort
    :heuristics 20 ((object-type (eql :retail-item)))
  ;; it's actually not the effort but the slippage parameter,
  ;; because that's what donbot's gripper speaks...
  0.72)

(defmethod man-int:get-action-gripping-effort
    :heuristics 20 ((object-type (eql :dish-washer-tabs)))
  0.2)
(defmethod man-int:get-action-gripping-effort
    :heuristics 20 ((object-type (eql :balea-bottle)))
  0.72)
(defmethod man-int:get-action-gripping-effort
    :heuristics 20 ((object-type (eql :deodorant)))
  0.9)
(defmethod man-int:get-action-gripping-effort
    :heuristics 20 ((object-type (eql :juice-box)))
  0.82)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod man-int:get-object-type-carry-config :heuristics 20
    ((object-type (eql :retail-item)) grasp)
  :carry)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; DISH-WASHER-TABS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *dish-washer-tabs-grasp-x-offset* 0.0 "in meters")
(defparameter *dish-washer-tabs-grasp-z-offset* 0.0 "in meters")
(defparameter *dish-washer-tabs-top-grasp-z-offset* 0.05 "in meters")
(defparameter *dish-washer-tabs-pregrasp-x-offset* 0.10 "in meters") ; 0.3
(defparameter *dish-washer-tabs-small-lift-z-offset* 0.01 "in meters")
(defparameter *dish-washer-tabs-lift-z-top-grasp-offset* 0.10 "in meters")
(defparameter *dish-washer-tabs-lift-z-other-grasp-offset* 0.05 "in meters")
(defparameter *dish-washer-tabs-lift-z-basket-offset* 0.10 "in meters")
(defparameter *dish-washer-tabs-2nd-lift-z-basket-offset* 0.31 "in meters")

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms :dish-washer-tabs '(:left :right) :top
  :grasp-translation `(0.0
                       0.0
                       ,*dish-washer-tabs-top-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0
                      0.0
                      ,(+ *dish-washer-tabs-top-grasp-z-offset*
                          *dish-washer-tabs-lift-z-top-grasp-offset*))
  :2nd-pregrasp-offsets `(0.0
                          0.0
                          ,(+ *dish-washer-tabs-top-grasp-z-offset*
                              *dish-washer-tabs-small-lift-z-offset*))
  :lift-translation `(0.0
                      0.0
                      ,(+ *dish-washer-tabs-top-grasp-z-offset*
                          *dish-washer-tabs-small-lift-z-offset*))
  :2nd-lift-translation `(0.0
                          0.0
                          ,(+ *dish-washer-tabs-top-grasp-z-offset*
                              *dish-washer-tabs-lift-z-top-grasp-offset*)))

;; BACK grasp robot
(man-int:def-object-type-to-gripper-transforms :dish-washer-tabs '(:left :right) :back
  :location-type :robot
  :grasp-translation `(,(- *dish-washer-tabs-grasp-x-offset*)
                        0.0
                        ,*dish-washer-tabs-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation-2*
  :pregrasp-offsets `(,(- *dish-washer-tabs-pregrasp-x-offset*)
                       0.0
                       ,(+ *dish-washer-tabs-grasp-z-offset*
                           *dish-washer-tabs-lift-z-other-grasp-offset*))
  :2nd-pregrasp-offsets `(,(- *dish-washer-tabs-pregrasp-x-offset*)
                           0.0
                           ,(+ *dish-washer-tabs-grasp-z-offset*
                               *dish-washer-tabs-small-lift-z-offset*))
  :lift-translation `(0.0
                      0.0
                      ,(+ *dish-washer-tabs-grasp-z-offset*
                          *dish-washer-tabs-small-lift-z-offset*))
  :2nd-lift-translation `(0.0
                          0.0
                          ,(+ *dish-washer-tabs-grasp-z-offset*
                              *dish-washer-tabs-lift-z-other-grasp-offset*)))

;; BACK grasp basket
;; (man-int:def-object-type-to-gripper-transforms :dish-washer-tabs '(:left :right) :back
;;   :location-type :basket
;;   :grasp-translation `(,(- *dish-washer-tabs-grasp-x-offset*)
;;                         0.0
;;                         ,*dish-washer-tabs-grasp-z-offset*)
;;   :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation-2*
;;   :pregrasp-offsets `(,(- 0.0
;;                           *dish-washer-tabs-grasp-x-offset*
;;                           *dish-washer-tabs-2nd-lift-z-basket-offset*)
;;                        0.0
;;                        ,*dish-washer-tabs-grasp-z-offset*)
;;   :2nd-pregrasp-offsets `(,(- 0.0
;;                               *dish-washer-tabs-grasp-x-offset*
;;                               *dish-washer-tabs-lift-z-basket-offset*)
;;                            0.0
;;                            ,*dish-washer-tabs-grasp-z-offset*)
;;   :lift-translation `(0.0
;;                       0.0
;;                       ,(+ *dish-washer-tabs-grasp-z-offset*
;;                           *dish-washer-tabs-lift-z-basket-offset*))
;;   :2nd-lift-translation `(0.0
;;                           0.0
;;                           ,(+ *dish-washer-tabs-grasp-z-offset*
;;                               *dish-washer-tabs-2nd-lift-z-basket-offset*
;;                               0.0)))
;; grasp
(defmethod man-int:get-object-type-to-gripper-transform
    ((object-type (eql :dish-washer-tabs))
     object-name
     arm
     (grasp (eql :back)))
  (make-arm-transform
   object-name arm
   (- *dish-washer-tabs-grasp-x-offset*)
   0.0
   *dish-washer-tabs-grasp-z-offset*
   man-int:*-x-across-z-grasp-rotation-2*))
;; pregrasp
(defmethod man-int:get-object-type-to-gripper-pregrasp-transforms
    ((type (eql :dish-washer-tabs))
     object-name
     arm
     (grasp (eql :back))
     (location (eql :basket))
     grasp-transform)
  (list
   (make-arm-transform
    object-name arm
    (- 0.0
       *dish-washer-tabs-grasp-x-offset*
       *dish-washer-tabs-2nd-lift-z-basket-offset*)
    0.0
    *dish-washer-tabs-grasp-z-offset*
    man-int:*-x-across-z-grasp-rotation-2*)
   (make-arm-transform
    object-name arm
    (- 0.0
       *dish-washer-tabs-grasp-x-offset*
       *dish-washer-tabs-lift-z-basket-offset*)
    0.0
    *dish-washer-tabs-grasp-z-offset*
    man-int:*-x-across-z-grasp-rotation-2*)
   (make-arm-transform
    object-name arm
    (- 0.0
       *dish-washer-tabs-grasp-x-offset*
       *dish-washer-tabs-lift-z-other-grasp-offset*)
    0.0
    *dish-washer-tabs-grasp-z-offset*
    man-int:*-x-across-z-grasp-rotation-2*)))
;; postgrasp
(defmethod man-int:get-object-type-wrt-base-frame-lift-transforms
    ((type (eql :dish-washer-tabs))
     arm
     (grasp (eql :back))
     (location (eql :basket)))
  (list
   (make-base-transform
    0.0
    0.0
    (+ *dish-washer-tabs-grasp-z-offset*
       *dish-washer-tabs-lift-z-other-grasp-offset*))
   (make-base-transform
    0.0
    0.0
    (+ *dish-washer-tabs-grasp-z-offset*
       *dish-washer-tabs-lift-z-basket-offset*))
   (make-base-transform
    0.0
    0.0
    (+ *dish-washer-tabs-grasp-z-offset*
       *dish-washer-tabs-2nd-lift-z-basket-offset*))))


;; FRONT grasp robot
(man-int:def-object-type-to-gripper-transforms :dish-washer-tabs '(:left :right) :front
  :location-type :robot
  :grasp-translation `(,*dish-washer-tabs-grasp-x-offset*
                       0.0
                       ,*dish-washer-tabs-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation-2*
  :pregrasp-offsets `(,*dish-washer-tabs-pregrasp-x-offset*
                      0.0
                      ,(+ *dish-washer-tabs-grasp-z-offset*
                          *default-retail-z-offset*))
  :2nd-pregrasp-offsets `(,*dish-washer-tabs-pregrasp-x-offset*
                          0.0
                          ,*dish-washer-tabs-grasp-z-offset*)
  :lift-translation `(0.0
                      0.0
                      ,(+ *dish-washer-tabs-grasp-z-offset*
                          *dish-washer-tabs-small-lift-z-offset*))
  :2nd-lift-translation `(0.0
                          0.0
                          ,(+ *dish-washer-tabs-grasp-z-offset*
                              *dish-washer-tabs-lift-z-other-grasp-offset*)))
;; FRONT grasp basket
(man-int:def-object-type-to-gripper-transforms :dish-washer-tabs '(:left :right) :front
  :location-type :basket
  :grasp-translation `(,*dish-washer-tabs-grasp-x-offset*
                       0.0
                       ,*dish-washer-tabs-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation-2*
  :pregrasp-offsets `(,(+ *dish-washer-tabs-pregrasp-x-offset*
                          *dish-washer-tabs-2nd-lift-z-basket-offset*)
                      0.0
                      ,*dish-washer-tabs-grasp-z-offset*)
  :2nd-pregrasp-offsets `(,(+ *dish-washer-tabs-pregrasp-x-offset*
                              *dish-washer-tabs-lift-z-basket-offset*)
                          0.0
                          ,*dish-washer-tabs-grasp-z-offset*)
  :lift-translation `(0.0
                      0.0
                      ,(+ *dish-washer-tabs-grasp-z-offset*
                          *dish-washer-tabs-lift-z-basket-offset*))
  :2nd-lift-translation `(0.0
                          0.0
                          ,(+ *dish-washer-tabs-grasp-z-offset*
                              *dish-washer-tabs-2nd-lift-z-basket-offset*)))

;; BACK grasp shelf
;; grasp
(defmethod man-int:get-object-type-to-gripper-transform
    ((object-type (eql :dish-washer-tabs))
     object-name
     arm
     (grasp (eql :back)))
  (make-arm-transform
   object-name arm
   (- *dish-washer-tabs-grasp-x-offset*)
   0.0
   *dish-washer-tabs-grasp-z-offset*
   man-int:*-x-across-z-grasp-rotation-2*))
;; pregrasp
(defmethod man-int:get-object-type-to-gripper-pregrasp-transforms
    ((type (eql :dish-washer-tabs))
     object-name
     arm
     (grasp (eql :back))
     location
     grasp-transform)
  (list
   (make-arm-transform
    object-name arm
    (- *dish-washer-tabs-pregrasp-x-offset*)
    0.0
    (+ *dish-washer-tabs-grasp-z-offset*
       *dish-washer-tabs-lift-z-other-grasp-offset*)
    man-int:*-x-across-z-grasp-rotation-2*)
   (make-arm-transform
    object-name arm
    (- *dish-washer-tabs-pregrasp-x-offset*)
    0.0
    (+ *dish-washer-tabs-grasp-z-offset*
       *dish-washer-tabs-lift-z-other-grasp-offset*)
    man-int:*-x-across-z-grasp-rotation-2*)
   (make-arm-transform
    object-name arm
    (- *dish-washer-tabs-pregrasp-x-offset*)
    0.0
    (+ *dish-washer-tabs-grasp-z-offset*
       *dish-washer-tabs-small-lift-z-offset*)
    man-int:*-x-across-z-grasp-rotation-2*)))
;; postgrasp
(defmethod man-int:get-object-type-wrt-base-frame-lift-transforms
    ((type (eql :dish-washer-tabs))
     arm
     (grasp (eql :back))
     location)
  (list
   (make-base-transform
    0.0
    0.0
    (+ *dish-washer-tabs-grasp-z-offset*
       *dish-washer-tabs-small-lift-z-offset*))
   (make-base-transform
    0.0
    0.0
    (+ *dish-washer-tabs-grasp-z-offset*
       *dish-washer-tabs-lift-z-other-grasp-offset*))
   (make-base-transform
    (- *dish-washer-tabs-pregrasp-x-offset*)
    0.0
    (+ *dish-washer-tabs-grasp-z-offset*
       *dish-washer-tabs-lift-z-other-grasp-offset*))))


;; FRONT grasp shelf
;; grasp
(defmethod man-int:get-object-type-to-gripper-transform
    ((object-type (eql :dish-washer-tabs))
     object-name
     arm
     (grasp (eql :front)))
  (make-arm-transform
   object-name arm
   *dish-washer-tabs-grasp-x-offset*
   0.0
   *dish-washer-tabs-grasp-z-offset*
   man-int:*x-across-z-grasp-rotation-2*))
;; pregrasp
(defmethod man-int:get-object-type-to-gripper-pregrasp-transforms
    ((type (eql :dish-washer-tabs))
     object-name
     arm
     (grasp (eql :front))
     location
     grasp-transform)
  (list
   (make-arm-transform
    object-name arm
    *dish-washer-tabs-pregrasp-x-offset*
    0.0
    (+ *dish-washer-tabs-grasp-z-offset*
       *dish-washer-tabs-lift-z-other-grasp-offset*)
    man-int:*x-across-z-grasp-rotation-2*)
   (make-arm-transform
    object-name arm
    *dish-washer-tabs-pregrasp-x-offset*
    0.0
    (+ *dish-washer-tabs-grasp-z-offset*
        *dish-washer-tabs-lift-z-other-grasp-offset*)
    man-int:*x-across-z-grasp-rotation-2*)
   (make-arm-transform
    object-name arm
    *dish-washer-tabs-pregrasp-x-offset*
    0.0
    (+ *dish-washer-tabs-grasp-z-offset*
       *dish-washer-tabs-small-lift-z-offset*)
    man-int:*x-across-z-grasp-rotation-2*)))
;; postgrasp
(defmethod man-int:get-object-type-wrt-base-frame-lift-transforms
    ((type (eql :dish-washer-tabs))
     arm
     (grasp (eql :front))
     location)
  (list
   (make-base-transform
    0.0
    0.0
    (+ *dish-washer-tabs-grasp-z-offset*
       *dish-washer-tabs-small-lift-z-offset*))
   (make-base-transform
    0.0
    0.0
    (+ *dish-washer-tabs-grasp-z-offset*
       *dish-washer-tabs-lift-z-other-grasp-offset*))
   (make-base-transform
    *dish-washer-tabs-pregrasp-x-offset*
    0.0
    (+ *dish-washer-tabs-grasp-z-offset*
       *dish-washer-tabs-lift-z-other-grasp-offset*))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; BALEA-BOTTLE ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *balea-bottle-grasp-z-offset* 0.0 "in meters")
(defparameter *balea-bottle-top-grasp-z-offset* 0.05 "in meters")
(defparameter *balea-bottle-pregrasp-x-offset* 0.10 "in meters")

;; TOP grasp
;; (man-int:def-object-type-to-gripper-transforms
;;     '(:balea-bottle
;;       :denkmit-entkalker
;;       :heitmann-citronensaeure
;;       :kuehne-essig-essenz
;;       :domestos-allzweckreiniger)
;;     '(:left :right) :top
;;   :grasp-translation `(0.0
;;                        0.0
;;                        ,*balea-bottle-top-grasp-z-offset*)
;;   :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
;;   :pregrasp-offsets *default-retail-lift-offsets*
;;   :2nd-pregrasp-offsets *default-retail-lift-offsets*
;;   :lift-translation *default-retail-lift-offsets*
;;   :2nd-lift-translation *default-retail-lift-offsets*)

;; BACK grasp
(man-int:def-object-type-to-gripper-transforms
    '(:balea-bottle
      :denkmit-entkalker
      :heitmann-citronensaeure
      :kuehne-essig-essenz
      :domestos-allzweckreiniger)
    '(:left :right) :back
  :grasp-translation `(0.0
                       0.0
                       ,*balea-bottle-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation-2*
  :pregrasp-offsets `(,(- *balea-bottle-pregrasp-x-offset*)
                       0.0
                       ,*default-retail-z-offset*)
  :2nd-pregrasp-offsets `(,(- *dish-washer-tabs-pregrasp-x-offset*)
                           0.0
                           ,*balea-bottle-grasp-z-offset*)
  :lift-translation *default-retail-lift-offsets*
  :2nd-lift-translation *default-retail-lift-offsets*)

;; FRONT grasp
(man-int:def-object-type-to-gripper-transforms
    '(:balea-bottle
      :denkmit-entkalker
      :heitmann-citronensaeure
      :kuehne-essig-essenz
      :domestos-allzweckreiniger)
    '(:left :right) :front
  :grasp-translation `(0.0
                       0.0
                       ,*balea-bottle-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation-2*
  :pregrasp-offsets `(,*balea-bottle-pregrasp-x-offset*
                      0.0
                      ,*default-retail-z-offset*)
  :2nd-pregrasp-offsets `(,*balea-bottle-pregrasp-x-offset*
                          0.0
                          ,*balea-bottle-grasp-z-offset*)
  :lift-translation *default-retail-lift-offsets*
  :2nd-lift-translation *default-retail-lift-offsets*)

;; LEFT-SIDE grasp
(man-int:def-object-type-to-gripper-transforms
    '(:heitmann-citronensaeure :domestos-allzweckreiniger) '(:left :right) :left-side
  :grasp-translation `(0.0
                       0.0
                       ,*balea-bottle-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0
                      ,*balea-bottle-pregrasp-x-offset*
                      ,*default-retail-z-offset*)
  :2nd-pregrasp-offsets `(0.0
                          ,*balea-bottle-pregrasp-x-offset*
                          ,*balea-bottle-grasp-z-offset*)
  :lift-translation *default-retail-lift-offsets*
  :2nd-lift-translation *default-retail-lift-offsets*)

;; RIGHT-SIDE grasp
(man-int:def-object-type-to-gripper-transforms
    '(:heitmann-citronensaeure :domestos-allzweckreiniger) '(:left :right) :right-side
  :grasp-translation `(0.0
                       0.0
                       ,*balea-bottle-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0
                      ,(- *balea-bottle-pregrasp-x-offset*)
                      ,*default-retail-z-offset*)
  :2nd-pregrasp-offsets `(0.0
                          ,(- *balea-bottle-pregrasp-x-offset*)
                          ,*balea-bottle-grasp-z-offset*)
  :lift-translation *default-retail-lift-offsets*
  :2nd-lift-translation *default-retail-lift-offsets*)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; DOMESTOS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *domestos-allzweckreiniger-grasp-z-offset* 0.035 "in meters")

;; BACK grasp
(man-int:def-object-type-to-gripper-transforms :domestos-allzweckreiniger '(:left :right) :back
  :grasp-translation `(0.0 0.0 ,*domestos-allzweckreiniger-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation-2*
  :pregrasp-offsets `(,(- *balea-bottle-pregrasp-x-offset*)
                       0.0
                       ,*default-retail-z-offset*)
  :2nd-pregrasp-offsets `(,(- *dish-washer-tabs-pregrasp-x-offset*)
                           0.0
                           ,*balea-bottle-grasp-z-offset*)
  :lift-translation *default-retail-lift-offsets*
  :2nd-lift-translation *default-retail-lift-offsets*)

;; FRONT grasp
(man-int:def-object-type-to-gripper-transforms :domestos-allzweckreiniger '(:left :right) :front
  :grasp-translation `(0.0 0.0 ,*domestos-allzweckreiniger-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation-2*
  :pregrasp-offsets `(,*balea-bottle-pregrasp-x-offset*
                      0.0
                      ,*default-retail-z-offset*)
  :2nd-pregrasp-offsets `(,*balea-bottle-pregrasp-x-offset*
                          0.0
                          ,*balea-bottle-grasp-z-offset*)
  :lift-translation *default-retail-lift-offsets*
  :2nd-lift-translation *default-retail-lift-offsets*)

;; LEFT-SIDE grasp
(man-int:def-object-type-to-gripper-transforms
    '(:heitmann-citronensaeure :domestos-allzweckreiniger) '(:left :right) :left-side
  :grasp-translation `(0.0 0.0 ,*domestos-allzweckreiniger-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0
                      ,*balea-bottle-pregrasp-x-offset*
                      ,*default-retail-z-offset*)
  :2nd-pregrasp-offsets `(0.0
                          ,*balea-bottle-pregrasp-x-offset*
                          ,*balea-bottle-grasp-z-offset*)
  :lift-translation *default-retail-lift-offsets*
  :2nd-lift-translation *default-retail-lift-offsets*)

;; RIGHT-SIDE grasp
(man-int:def-object-type-to-gripper-transforms
    '(:heitmann-citronensaeure :domestos-allzweckreiniger) '(:left :right) :right-side
  :grasp-translation `(0.0 0.0 ,*domestos-allzweckreiniger-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0
                      ,(- *balea-bottle-pregrasp-x-offset*)
                      ,*default-retail-z-offset*)
  :2nd-pregrasp-offsets `(0.0
                          ,(- *balea-bottle-pregrasp-x-offset*)
                          ,*balea-bottle-grasp-z-offset*)
  :lift-translation *default-retail-lift-offsets*
  :2nd-lift-translation *default-retail-lift-offsets*)



;;;;;;;;;;;;;;; DENKMIT, DOVE, HEITMANN and SOMAT ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *denkmit-pregrasp-xy-offste* 0.3 "in meters")
(defparameter *denkmit-grasp-xy-offset* 0.03 "in meters")
(defparameter *denkmit-grasp-z-offset* 0.03 "in meters")

(man-int:def-object-type-to-gripper-transforms :denkmit '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,*denkmit-grasp-xy-offset* ,*denkmit-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* ,*default-retail-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* 0.0)
  :lift-translation *default-retail-lift-offsets*
  :2nd-lift-translation *default-retail-lift-offsets*)

(man-int:def-object-type-to-gripper-transforms :dove '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,*denkmit-grasp-xy-offset* ,*denkmit-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* ,*default-retail-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* 0.0)
  :lift-translation *default-retail-lift-offsets*
  :2nd-lift-translation *default-retail-lift-offsets*)

(man-int:def-object-type-to-gripper-transforms :heitmann '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,*denkmit-grasp-xy-offset* ,*denkmit-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* ,*default-retail-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* 0.0)
  :lift-translation *default-retail-lift-offsets*
  :2nd-lift-translation *default-retail-lift-offsets*)

(man-int:def-object-type-to-gripper-transforms :somat '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,*denkmit-grasp-xy-offset* ,*denkmit-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* ,*default-retail-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* 0.0)
  :lift-translation *default-retail-lift-offsets*
  :2nd-lift-translation *default-retail-lift-offsets*)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; BASKET ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(man-int:def-object-type-to-gripper-transforms :basket '(:left :right) :top
  :grasp-translation `(0.15 0.0 0.18)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets *default-retail-lift-offsets*
  :2nd-pregrasp-offsets *default-retail-lift-offsets*
  :lift-translation *default-retail-lift-offsets*
  :2nd-lift-translation *default-retail-lift-offsets*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;; placing poses on robot, environment and basket

(man-int:def-object-type-in-other-object-transform :dish-washer-tabs :robot
  :donbot-tray-front
  :attachment-translation `(0.2 0.05 0.11)
  :attachment-rot-matrix '((0  0  1)
                           (0 -1  0)
                           (1  0  0)))

(man-int:def-object-type-in-other-object-transform :dish-washer-tabs :robot
  :donbot-tray-back
  :attachment-translation `(0.2 0.05 0.11)
  :attachment-rot-matrix '(( 0  0  1)
                           ( 0  1  0)
                           (-1  0  0)))

(man-int:def-object-type-in-other-object-transform :dish-washer-tabs :environment
  :dish-washer-tabs-shelf-1-front
  :attachment-translation `(0.39968 -0.26038335 0.10) ; 0.1202
  :attachment-rot-matrix man-int:*rotation-around-z-90-matrix*)

(man-int:def-object-type-in-other-object-transform :dish-washer-tabs :environment
  :dish-washer-tabs-shelf-1-back
  :attachment-translation `(0.39968 -0.26038335 0.10) ; 0.1202
  :attachment-rot-matrix man-int:*rotation-around-z+90-matrix*)

(man-int:def-object-type-in-other-object-transform :dish-washer-tabs :basket
  :in-basket-front
  :attachment-translation `(0.15 0.15 -0.02)
  :attachment-rot-matrix '(( 0  0  1)
                           ( 0  1  0)
                           (-1  0  0)))

(man-int:def-object-type-in-other-object-transform :dish-washer-tabs :basket
  :in-basket-back
  :attachment-translation `(0.15 0.15 -0.02)
  :attachment-rot-matrix '(( 0  0 -1)
                           ( 0 -1  0)
                           (-1  0  0)))
(man-int:def-object-type-in-other-object-transform :dish-washer-tabs :basket
  :in-basket-other-front
  :attachment-translation `(0.15 -0.15 -0.02)
  :attachment-rot-matrix '(( 0  0  1)
                           ( 0  1  0)
                           (-1  0  0)))

(man-int:def-object-type-in-other-object-transform :dish-washer-tabs :basket
  :in-basket-other-back
  :attachment-translation `(0.15 -0.15 -0.02)
  :attachment-rot-matrix '(( 0  0 -1)
                           ( 0 -1  0)
                           (-1  0  0)))

(man-int:def-object-type-in-other-object-transform :balea-bottle :environment
  :balea-bottle-shelf-1-front
  :attachment-translation `(0.3 -0.27 0.105)
  :attachment-rot-matrix man-int:*rotation-around-z-90-matrix*)

(man-int:def-object-type-in-other-object-transform :balea-bottle :environment
  :balea-bottle-shelf-1-back
  :attachment-translation `(0.3 -0.27 0.105)
  :attachment-rot-matrix man-int:*rotation-around-z+90-matrix*)

(man-int:def-object-type-in-other-object-transform :balea-bottle :robot
  :donbot-tray-left
  :attachment-translation `(0.2 0.1 0.1)
  :attachment-rot-matrix '((0.43809 0.89892 0.005072)
                           (-0.89879 0.43811461 -0.01522)
                           (-0.0159 0.0021 0.999871)))

(man-int:def-object-type-in-other-object-transform :heitmann :basket
  :in-basket
  :attachment-translation `(0.2 0.15 -0.005)
  :attachment-rot-matrix '((1 0 0)
                           (0 1 0)
                           (0 0 1)))

(man-int:def-object-type-in-other-object-transform :dove :basket
  :in-basket
  :attachment-translation `(0.1 0.15 -0.005)
  :attachment-rot-matrix '((1 0 0)
                           (0 1 0)
                           (0 0 1)))





(man-int:def-object-type-in-other-object-transform :dish-washer-tabs :robot
  :kukabot-tray-front
  :attachment-translation `(0.1 0.1 0.75)
  :attachment-rot-matrix '((0  0  1)
                           (0 -1  0)
                           (1  0  0)))

(man-int:def-object-type-in-other-object-transform :dish-washer-tabs :robot
  :kukabot-tray-back
  :attachment-translation `(0.1 0.1 0.75)
  :attachment-rot-matrix '(( 0  0  1)
                           ( 0  1  0)
                           (-1  0  0)))

(man-int:def-object-type-in-other-object-transform :dish-washer-tabs :environment
  :dish-washer-tabs-real-shelf-1-front
  :attachment-translation `(0.39968 -0.26038335 0.1902)
  :attachment-rot-matrix man-int:*rotation-around-z-90-matrix*)

(man-int:def-object-type-in-other-object-transform :dish-washer-tabs :environment
  :dish-washer-tabs-real-shelf-1-back
  :attachment-translation `(0.39968 -0.26038335 0.1902)
  :attachment-rot-matrix man-int:*rotation-around-z+90-matrix*)

(man-int:def-object-type-in-other-object-transform :balea-bottle :environment
  :balea-bottle-real-shelf-1-front
  :attachment-translation `(;0.33 -0.27 0.175
                            -0.05 -0.27 0.175)
  :attachment-rot-matrix man-int:*rotation-around-z-90-matrix*)

(man-int:def-object-type-in-other-object-transform :balea-bottle :environment
  :balea-bottle-real-shelf-1-back
  :attachment-translation `(;0.33 -0.27 0.175
                            -0.05 -0.27 0.175)
  :attachment-rot-matrix man-int:*rotation-around-z+90-matrix*)



(defmethod man-int:get-z-offset-for-placing-with-dropping (object
                                                           (other-object (eql :basket))
                                                           attachment)
  0.0 ;; 0.15 ; <- we have to pick it back up, so no point in dropping stuff
  )
