(in-package :cram-manipulation-interfaces)

;currently in trajectories.lisp
;(defmethod man-int:get-object-type-robot-frame-whisk-approach-transform
;    ((object-type (eql :big-bowl))
;     arm)
                                        ;  '((0.0 0.0 0.3)(0 0.707 0 0.707)))

(defmethod get-object-type-robot-frame-mix-grip-approach-transform
    ((object-type (eql :big-bowl))
     (arm (eql :left))
     (grasp (eql :top)))
   '((0 -0.12  0.161)(1 0 0 0)))


(defmethod get-object-type-robot-frame-mix-grip-approach-transform
    ((object-type (eql :saucepan))
     (arm (eql :left))
     (grasp (eql :top)))
   '((0.153 0.025 0.093)(1 0 0 0)))
;0.153 cos radius + lenght to handle middle <- tina todo pls look at numbers x yz axis again
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod get-object-type-robot-frame-mix-grip-retract-transform
    ((object-type (eql :big-bowl))
     (arm (eql :left))
     (grasp (eql :top)))
   '((0 -0.04  0.6)(1 0 0 0))) ;0.02 radius, -0.12 depth, 0.06 height

(defmethod get-object-type-robot-frame-mix-grip-retract-transform
    ((object-type (eql :saucepan))
     (arm (eql :left))
     (grasp (eql :top)))
  '((0.093 0.093 0.04)(1 0 0 0)))

(defmethod get-object-type-robot-frame-mix-grip-retract-transform
    ((object-type (eql :saucepan))
     (arm (eql :right))
     (grasp (eql :top)))
    '((0 0 0.093)(1 0 0 0)))
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; don't care for above as long as no grip implemented

(defmethod get-object-type-robot-frame-mix-grip-container-transform
   ( (object-type (eql :big-bowl))
  (container-arm (eql :left))
     (grasp (eql :left-side)))
  (print "yes- this was used.")
      '((0 0 0.093)(1 0 0 0)))

(defmethod get-object-type-robot-frame-mix-grip-container-transform
   ( (object-type (eql :big-bowl))
  (container-arm (eql :right))
(grasp (eql :right-side)))
      '((0 0 0.093)(1 0 0 0)))

;; (defmethod get-object-type-robot-frame-mix-grip-container-transform
;;     (object-type
;;       tool-object-type
;;       container-arm)
;;       '((0 0 0.093)(1 0 0 0)))
;; (defmethod get-object-type-robot-frame-mix-grip-container-transform
;;     (object-type
;;       tool-object-type
;;       container-arm)
;;       '((0 0 0.093)(1 0 0 0)))
;; (defmethod get-object-type-robot-frame-mix-grip-container-transform
;;     (object-type
;;       tool-object-type
;;       container-arm)
;;       '((0 0 0.093)(1 0 0 0)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;approach pose in bto

(defmethod get-object-type-robot-frame-mix-approach-transform
    ((object-type (eql :big-bowl))
      (arm (eql :right))
     (grasp (eql :top)))
  '((0 0 0)(1 0 0 0)));0.02 -0.12 0.161)(1 0 0 0)))

(defmethod get-object-type-robot-frame-mix-approach-transform
    ((object-type (eql :saucepan))
      (arm (eql :right))
     (grasp (eql :top)))
  '((0 0 0)(1 0 0 0)))
  
(defmethod get-object-type-robot-frame-mix-approach-transform
    ((object-type (eql :bowl-round))
      (arm (eql :right))
     (grasp (eql :top)))
  '((0 0 0)(1 0 0 0)))

(defmethod get-object-type-robot-frame-mix-approach-transform
    ((object-type (eql :wine-glas))
      (arm (eql :right))
     (grasp (eql :top)))
  '((0 0 0)(1 0 0 0)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;retract pose in bto z= depth

;; (defmethod get-object-type-robot-frame-mix-retract-transform
;;      ((object-type (eql :big-bowl))
;;       (arm (eql :right))
;;      (grasp (eql :top)))
;;   '((0 0 0.06)(1 0 0 0)))   

;; (defmethod get-object-type-robot-frame-mix-retract-transform
;;      ((object-type (eql :saucepan))
;;       (arm (eql :right))
;;       (grasp (eql :top)))
;;   '((0 0 0.04)(1 0 0 0)))
  
;; (defmethod get-object-type-robot-frame-mix-retract-transform
;;      ((object-type (eql :bowl-round))
;;       (arm (eql :right))
;;       (grasp (eql :top)))
;;   '((0 0 0.025)(1 0 0 0)))

;; (defmethod get-object-type-robot-frame-mix-retract-transform
;;      ((object-type (eql :wine-glas))
;;       (arm (eql :right))
;;       (grasp (eql :top)))
;;   '((0 0 0.09)(1 0 0 0)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;in object coordiantes mix information for upper circle
;((toprim z, radius, origin z )(rotation))

(defmethod get-object-type-robot-frame-mix-rim-top-transform
   ((object-type (eql :big-bowl)))
  '((0.06 0.12 0)(1 0 0 0)))
;0.06 0.12 - radius 0.12 top, height 0.06

(defmethod get-object-type-robot-frame-mix-rim-top-transform
   ((object-type (eql :saucepan)))
  '((0.093 0.09 0)(1 0 0 0)))

(defmethod get-object-type-robot-frame-mix-rim-top-transform
   ((object-type (eql :bowl-round)))
  '((0.057 0.03 0)(1 0 0 0)))

(defmethod get-object-type-robot-frame-mix-rim-top-transform
   ((object-type (eql :wine-glas)))
  '((0.1 0.2 0.05)(1 0 0 0)))
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;in object coordinates mix information for lower circle

(defmethod get-object-type-robot-frame-mix-rim-bottom-transform
   ((object-type (eql :big-bowl)))
   '((0.055 0.06 -0.045)(1 0 0 0))) ;16 cm diameter 6 cm below origin


(defmethod get-object-type-robot-frame-mix-rim-bottom-transform
   ((object-type (eql :saucepan)))
  '((0.093 0.09 -0.04)(1 0 0 0)))
  
(defmethod get-object-type-robot-frame-mix-rim-bottom-transform
   ((object-type (eql :bowl-round)))
  '((0.06 0.04 -0.023)(1 0 0 0)))

(defmethod get-object-type-robot-frame-mix-rim-bottom-transform
   ((object-type (eql :wine-glas)))
  '((0.03 0.01 0.005)(1 0 0 0)))

;decided to use the z axis as radius measure (important for mix center point calculation)

;(topheight, radius, bottomheight )(rotation) - think of the thickness!
; origin height radius will be calc with pythagoras

;for top rim look at mix-approach-transform

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; utensils
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;-bowl bottom can be looked up in rim-bottom-transform 

;(lenght from origin to tip of the tool going into the container, width, depth)(rotation)

(defmethod get-object-type-robot-frame-mix-tool-transform
    ((object-type (eql :whisk))
     )
  '((0.15 0.03 0.03)(1 0 0 0)))

 (defmethod get-object-type-robot-frame-mix-tool-transform
    ((object-type (eql :fork))
     )
  '((0.11 0.02 0.015)(1 0 0 0)))

 (defmethod get-object-type-robot-frame-mix-tool-transform
    ((object-type (eql :ladle))
     )
   '((0.13 0.044 0.044)(1 0 0 0)))

 (defmethod get-object-type-robot-frame-mix-tool-transform
    ((object-type (eql :tea-spoon))
     )
  '((0.085 0.02 0.01)(1 0 0 0)))
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                                        ;snatched back from household

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; big-bowl ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *big-bowl-grasp-xy-offset* 0.12 "in meters")
(defparameter *big-bowl-pregrasp-z-offset* 0.20 "in meters")
(defparameter *big-bowl-grasp-z-offset* 0.05);0.12 "in meters") ;height
(defparameter *big-bowl-grasp-thickness* 0.01 "in meter")
;big brain move- toate the axis to fit with whisk and co
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
  :pregrasp-offsets `(-0.05 0 0.2)
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
(man-int:def-object-type-to-gripper-transforms '(:saucepan) '(:left :right) :handle-left
  :grasp-translation `(-0.2,0.0 ,*saucepan-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*saucepan-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0, 0.0 ,*saucepan-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 ,0.0)
  :2nd-lift-translation `(0.0 0.0 ,0.0))

(man-int:def-object-type-to-gripper-transforms '(:saucepan) '(:left :right) :handle-right
  :grasp-translation `(-0.2,0.0 ,*saucepan-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*saucepan-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0, 0.0 ,*saucepan-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 ,0.0)
  :2nd-lift-translation `(0.0 0.0 ,0.0))

(man-int:def-object-type-to-gripper-transforms '(:saucepan) '(:left :right) :handle-top
  :grasp-translation `(-0.2,0.0 ,*saucepan-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*saucepan-pregrasp-z-offset*)
  :2nd-pregrasp-offsets `(0.0, 0.0 ,*saucepan-pregrasp-z-offset*)
  :lift-translation `(0.0 0.0 ,0.0)
  :2nd-lift-translation `(0.0 0.0 ,0.0))

(man-int:def-object-type-to-gripper-transforms '(:saucepan) '(:left :right) :handle-bottom
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

;; TOP grasp
(man-int:def-object-type-to-gripper-transforms :wine-glas '(:left :right) :stem-left
  :grasp-translation `(0.0 0.0 ,*wine-glas-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*wine-glas-lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*wine-glas-lift-z-offset*)
  :lift-translation `(0.0 0.0 ,0.0)
  :2nd-lift-translation `( 0.01 0.0 ,0.0))

(man-int:def-object-type-to-gripper-transforms :wine-glas '(:left :right) :stem-right
  :grasp-translation `(0.0 0.0 ,*wine-glas-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*wine-glas-lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*wine-glas-lift-z-offset*)
  :lift-translation `(0.0 0.0 ,0.0)
  :2nd-lift-translation `( 0.01 0.0 ,0.0))

(man-int:def-object-type-to-gripper-transforms :wine-glas '(:left :right) :stem-front
  :grasp-translation `(0.0 0.0 ,*wine-glas-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,*wine-glas-lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 0.0 ,*wine-glas-lift-z-offset*)
  :lift-translation `(0.0 0.0 ,0.0)
  :2nd-lift-translation `( 0.01 0.0 ,0.0))
