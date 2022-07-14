(in-package :pr2-wipe)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; sponge when its done ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;wiping offsets;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


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
