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
   '((0.02 -0.12  0.2)(1 0 0 0)))

(defmethod get-object-type-robot-frame-mix-grip-retract-transform
    ((object-type (eql :saucepan))
     (arm (eql :left))
     (grasp (eql :top)))
  '((0.093 0.093 0.04)(1 0 0 0)))
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defmethod get-object-type-robot-frame-mix-approach-transform
    ((object-type (eql :big-bowl))
      (arm (eql :right))
     (grasp (eql :top)))
  '((0.02 -0.12 0.161)(1 0 0 0)))

(defmethod get-object-type-robot-frame-mix-approach-transform
    ((object-type (eql :saucepan))
      (arm (eql :right))
     (grasp (eql :top)))
  '((0.093 0.093 0.04)(1 0 0 0)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod get-object-type-robot-frame-mix-retract-transform
     ((object-type (eql :big-bowl))
      (arm (eql :right))
     (grasp (eql :top)))
  '((0.02 -0.12 0.28)(1 0 0 0)))   

(defmethod get-object-type-robot-frame-mix-retract-transform
     ((object-type (eql :saucepan))
      (arm (eql :right))
     (grasp (eql :top)))
  '((0.093 0.093 0.04)(1 0 0 0)))

;;the z should be  defined by:
;;object in hand where?
;;how long is object from where gripper is
;;y is height

;should be defined in household later--cos 12 is too close to rim


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



(defmethod get-object-type-robot-frame-mix-grip-retract-transform
    ((object-type (eql :saucepan))
     (arm (eql :right))
     (grasp (eql :top)))
    '((0 0.04 0.093)(1 0 0 0)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod get-object-type-robot-frame-mix-retract-transform
     ((object-type (eql :saucepan))
      (arm (eql :right))
     (grasp (eql :top)))
  '((0.093 0.093 0.04)(1 0 0 0)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod get-object-type-robot-frame-mix-rim-bottom-transform
   ((object-type (eql :big-bowl)))
   '((0.0 -0.12 0.06)(1 0 0 0)))


(defmethod get-object-type-robot-frame-mix-rim-bottom-transform
   ((object-type (eql :saucepan)))
  '((0.093 0.093 -0.04)(1 0 0 0)))

;decided to use the z axis as radius measure (important for mix center point calculation)
;for top rim look at mix-approach-transform

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod get-object-type-robot-frame-mix-rim-grip-deep-approach-transform
    ((object-type (eql :big-bowl))
        (arm (eql :right))
     (grasp (eql :top)))
  '((0.0 -0.12 0.15)(1 0 0 0)))

;;height depending on tool...center to bottom measurment
;;-bowl bottom can be looked up in rim-bottom-transform ^
;decided y is height/lenght of object from center grip

(defmethod get-object-type-robot-frame-mix-rim-tool-grip-bottom-transform
    ((object-type (eql :whisk)))
  '((0.03 0.03 0.15)(1 0 0 0)))
                                        ; '((0.02 -0.12 0.06)(1 0 0 0)))

;decided to use the z axis as radius measure (important for mix center point calculation)
;decided y to be height og bowl and ground of bowl - need to calc with whisk dimensions to get robot gripper position in object tf before calc on higher level

(defmethod get-object-type-robot-frame-mix-rim-top-transform
   ((object-type (eql :big-bowl)))
  '((0.0 -0.09 0.11)(1 0 0 0))) ;0.11

(defmethod get-object-type-robot-frame-mix-tool-transform
    ((object-type (eql :whisk))
     )
  '((0.03 0.03 0.15)(1 0 0 0)))

   
 ; '((0.03 0.145 0.015)(1 0 0 0)))

;why did I thought that 0.3 would be a good x ?? asking past tina . it gotta be 0.14 says my  19.02. brain - nohooo 28.05. tina brain answer: 0.03 is the whisk hair width on x axis (so diameter of 0.06), the 0.14 is for lenght of (correction 8.6.:) the whole whisk from center to tip of the whisk; (11.06.) original pat tina was right with 0.03 as x.
