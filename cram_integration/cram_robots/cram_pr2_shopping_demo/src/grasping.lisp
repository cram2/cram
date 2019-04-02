(in-package :kr-pp)


(defparameter *lift-z-offset* 0.05 "in meters")

(defmethod get-object-type-grasping-effort ((object-type (eql :denkmit ))) 50)

;;;;;;;;;;;;;;;;;;;;;;;;;;; Denkmit ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *denkmit-pregrasp-xy-offste* 0.00 "in meters")
(defparameter *denkmit-grasp-xy-offset* 0.00 "in meters")
(defparameter *denkmit-grasp-z-offset* 0.00 "in meters")

;;SIDE grasp
(def-object-type-to-gripper-transforms :denkmit '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,*denkmit-grasp-xy-offset* ,*denkmit-grasp-z-offset*)
  :grasp-rot-matrix *y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

(def-object-type-to-gripper-transforms :dove '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,*denkmit-grasp-xy-offset* ,*denkmit-grasp-z-offset*)
  :grasp-rot-matrix *y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

(def-object-type-to-gripper-transforms :heitmann '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,*denkmit-grasp-xy-offset* ,*denkmit-grasp-z-offset*)
  :grasp-rot-matrix *y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)

(def-object-type-to-gripper-transforms :somat '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,*denkmit-grasp-xy-offset* ,*denkmit-grasp-z-offset*)
  :grasp-rot-matrix *y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*denkmit-pregrasp-xy-offste* 0.0)
  :lift-offsets *lift-offset*
  :2nd-lift-offsets *lift-offset*)


;; (def-object-type-to-gripper-transforms :dove '(:left :right) :left-side
;;   :grasp-translation `(0.0d0 0.00 0.00)
;;   :grasp-rot-matrix *left-side-grasp-rotation*
;;   :pregrasp-offsets `(0.0 0.00 ,*lift-z-offset*)
;;   :2nd-pregrasp-offsets `(0.0 0.00 0.0)
;;   :lift-offsets *lift-z-offset*
;;   :2nd-lift-offsets *lift-z-offset*)


;; (def-object-type-to-gripper-transforms :denkmit '(:left :right) :left-side
;;   :lift-offsets *lift-z-offset*
;;   :2nd-lift-offsets *lift-z-offset*)
