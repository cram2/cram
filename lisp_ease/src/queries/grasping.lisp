(in-package :le)

;; orientation source: cram_knowrob_pick_place

;; grasping offsets -------------------------------------------------------------

(defparameter *lift-z-offset* 0.15 "in meters") ;0.15

(defparameter *muesli-grasp-z-offset* 0.13 "in meters") ; 0.13
(defparameter *muesli-grasp-xy-offset* -0.03 "in meters") ; -0.03
(defparameter *muesli-pregrasp-xy-offset* 0.0 "in meters")  ; 0.15

(defparameter *milk-grasp-xy-offset* -0.05 "in meters") ;-0.20
(defparameter *milk-grasp-z-offset* 0.0 "in meters")
(defparameter *milk-pregrasp-xy-offset* -0.05 "in meters") ;0.15

(defparameter *bowl-grasp-x-offset* 0.07 "in meters")   ;0.07
(defparameter *bowl-grasp-z-offset* 0.01 "in meters")   ;0.01
(defparameter *bowl-pregrasp-z-offset* 0.15 "in meters");0.15

(defparameter *cup-grasp-z-offset* -0.03 "in meters") ; 0.03
(defparameter *cup-grasp-xy-offset* 0.02 "in meters") ; 0.02
(defparameter *cup-pregrasp-xy-offset* -0.15 "in meters") ; 0.15

(defparameter *cutlery-grasp-z-offset* -0.0 "in meters") ; because TCP is not at the edge
(defparameter *cutlery-pregrasp-z-offset* 0.20 "in meters")

;; grasping force in Nm --------------------------------------------------------
(defmethod get-object-type-gripping-effort ((object-type (eql :ba-muesli))) 15)
(defmethod get-object-type-gripping-effort ((object-type (eql :ba-milk))) 15)
(defmethod get-object-type-gripping-effort ((object-type (eql :ba-bowl))) 15)
(defmethod get-object-type-gripping-effort ((object-type (eql :ba-cup))) 15)
(defmethod get-object-type-gripping-effort ((object-type (eql :ba-fork))) 100)

;; gripper oppening ------------------------------------------------------------
(defmethod get-object-type-gripper-opening ((object-type (eql :ba-muesli))) 0.2)
(defmethod get-object-type-gripper-opening ((object-type (eql :ba-milk))) 0.2)
(defmethod get-object-type-gripper-opening ((object-type (eql :ba-bowl))) 0.2)
(defmethod get-object-type-gripper-opening ((object-type (eql :ba-cup))) 0.2)
(defmethod get-object-type-gripper-opening ((object-type (eql :ba-fork))) 0.03)
(defmethod get-object-type-gripper-opening (object-type)
  "Default value is 0.10. In meters."
  0.10)
;;-------------------------------------------------------------------------------
(defmethod get-object-type-to-gripper-lift-transform (object-type object-name
                                                      arm grasp grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :z-offset *lift-z-offset*))

;; grasps ---------------------------------------------------------------------------------------
;------------------------------------------------------------------------------------------------
;;  MUESLI
;------------------------------------------------------------------------------------------------

(defmethod get-object-type-to-gripper-transform ((object-type (eql :ba-muesli))
                                                 object-name
                                                 arm
                                                 (grasp (eql :human-grasp)))
  (print "GRASPING STUFF LIKE A HUMAN.")
  (let* (transf
         end-transf)

    ;; transf. from Map to Obj?
    (setq transf
          (cl-tf:transform*
            (cl-tf:transform-inv
               (make-poses "?PoseObjStart"))
           (make-poses "?PoseHandStart")
           (human-to-robot-hand-transform)))
    
    (print "***human-grasp***")
    (setf end-transf
          (cl-tf:transform->transform-stamped
           (roslisp-utilities:rosify-underscores-lisp-name object-name)
           (ecase arm
             (:left cram-tf:*robot-left-tool-frame*)
             (:right cram-tf:*robot-right-tool-frame*))
           0.0
           transf))

    (print "------------------------------")
    (print "THIS IS THE END TRANSFORM:")
    (print "------------------------------")
    (print end-transf)
    end-transf))

(defmethod get-object-type-to-gripper-pregrasp-transform ((object-type (eql :ba-muesli))
                                                          object-name
                                                          arm
                                                          (grasp (eql :human-grasp))
                                                          grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *muesli-pregrasp-xy-offset*)
                                                       :z-offset *lift-z-offset*))
(defmethod get-object-type-to-gripper-2nd-pregrasp-transform ((object-type (eql :ba-muesli))
                                                              object-name
                                                              arm
                                                              (grasp (eql :human-grasp))
                                                              grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *muesli-pregrasp-xy-offset*)))
;------------------------------------------------------------------------------------------------
;; MILK -----------------------------------------------------------------------------------------
;------------------------------------------------------------------------------------------------
(defmethod get-object-type-to-gripper-transform ((object-type (eql :ba-milk))
                                                 object-name
                                                 arm
                                                 (grasp (eql :human-grasp)))
  (print "GRASPING STUFF LIKE A HUMAN.")
  (let* (transf
         end-transf)

    ;; transf. from Map to Obj?
    (setq transf
          (cl-tf:transform*
            (cl-tf:transform-inv
               (make-poses "?PoseObjStart"))
           (make-poses "?PoseHandStart")
           (human-to-robot-hand-transform)))
    
    (print "***human-grasp***")
    (setf end-transf
          (cl-tf:transform->transform-stamped
           (roslisp-utilities:rosify-underscores-lisp-name object-name)
           (ecase arm
             (:left cram-tf:*robot-left-tool-frame*)
             (:right cram-tf:*robot-right-tool-frame*))
           0.0
           transf))

    (print "------------------------------")
    (print "THIS IS THE END TRANSFORM:")
    (print "------------------------------")
    (print end-transf)
    end-transf))

(defmethod get-object-type-to-gripper-pregrasp-transform ((object-type (eql :ba-milk))
                                                          object-name
                                                          arm
                                                          (grasp (eql :human-grasp))
                                                          grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *milk-pregrasp-xy-offset*)
                                                       :z-offset *lift-z-offset*))


(defmethod get-object-type-to-gripper-2nd-pregrasp-transform ((object-type (eql :ba-milk))
                                                              object-name
                                                              arm
                                                              (grasp (eql :human-grasp))
                                                              grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *milk-pregrasp-xy-offset*)))



;------------------------------------------------------------------------------------------------
; BOWL ------------------------------------------------------------------------------------------
;------------------------------------------------------------------------------------------------
(defmethod get-object-type-to-gripper-transform ((object-type (eql :ba-bowl))
                                                 object-name
                                                 arm
                                                 (grasp (eql :human-grasp)))
  (print "GRASPING STUFF LIKE A HUMAN.")
  (let* (transf
         end-transf)

    ;; transf. from Map to Obj?
    (setq transf
          (cl-tf:transform*
            (cl-tf:transform-inv
               (make-poses "?PoseObjStart"))
           (make-poses "?PoseHandStart")
           (human-to-robot-hand-transform)))
    
    (print "***human-grasp***")
    (setf end-transf
          (cl-tf:transform->transform-stamped
           (roslisp-utilities:rosify-underscores-lisp-name object-name)
           (ecase arm
             (:left cram-tf:*robot-left-tool-frame*)
             (:right cram-tf:*robot-right-tool-frame*))
           0.0
           transf))

    (print "------------------------------")
    (print "THIS IS THE END TRANSFORM:")
    (print "------------------------------")
    (print end-transf)
    end-transf))

(defmethod get-object-type-to-gripper-pregrasp-transform ((object-type (eql :ba-bowl))
                                                          object-name
                                                          arm
                                                          (grasp (eql :human-grasp))
                                                          grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *bowl-pregrasp-z-offset*)
                                                       :z-offset *lift-z-offset*))


(defmethod get-object-type-to-gripper-2nd-pregrasp-transform ((object-type (eql :ba-bowl))
                                                              object-name
                                                              arm
                                                              (grasp (eql :human-grasp))
                                                              grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *bowl-pregrasp-z-offset*)))
;------------------------------------------------------------------------------------------------
; CUP -------------------------------------------------------------------------------------------
;------------------------------------------------------------------------------------------------

(defmethod get-object-type-to-gripper-transform ((object-type (eql :ba-cup))
                                                 object-name
                                                 arm
                                                 (grasp (eql :human-grasp)))
  (print "GRASPING STUFF LIKE A HUMAN.")
  (let* (transf
         end-transf)

    ;; transf. from Map to Obj?
    (setq transf
          (cl-tf:transform*
            (cl-tf:transform-inv
               (make-poses "?PoseObjStart"))
           (make-poses "?PoseHandStart")
           (human-to-robot-hand-transform)))
    
    (print "***human-grasp***")
    (setf end-transf
          (cl-tf:transform->transform-stamped
           (roslisp-utilities:rosify-underscores-lisp-name object-name)
           (ecase arm
             (:left cram-tf:*robot-left-tool-frame*)
             (:right cram-tf:*robot-right-tool-frame*))
           0.0
           transf))

    (print "------------------------------")
    (print "THIS IS THE END TRANSFORM:")
    (print "------------------------------")
    (print end-transf)
    end-transf))

(defmethod get-object-type-to-gripper-pregrasp-transform ((object-type (eql :ba-cup))
                                                          object-name
                                                          arm
                                                          (grasp (eql :human-grasp))
                                                          grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *cup-pregrasp-xy-offset*)
                                                       :z-offset *lift-z-offset*))
(defmethod get-object-type-to-gripper-2nd-pregrasp-transform ((object-type (eql :ba-cup))
                                                              object-name
                                                              arm
                                                              (grasp (eql :human-grasp))
                                                              grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *cup-pregrasp-xy-offset*)))


;;-----------------------------------------------------------------------------
;; FORK
;;----------------------------------------------------------------------------
(defmethod get-object-type-to-gripper-transform ((object-type (eql :ba-fork))
                                                 object-name
                                                 arm
                                                 (grasp (eql :human-grasp)))
  (print "GRASPING STUFF LIKE A HUMAN.")
  (let* (transf
         end-transf)

    ;; transf. from Map to Obj?
    (setq transf
          (cl-tf:transform*
            (cl-tf:transform-inv
               (make-poses "?PoseObjStart"))
           (make-poses "?PoseHandStart")
           (human-to-robot-hand-transform)))
    
    (print "***human-grasp***")
    (setf end-transf
          (cl-tf:transform->transform-stamped
           (roslisp-utilities:rosify-underscores-lisp-name object-name)
           (ecase arm
             (:left cram-tf:*robot-left-tool-frame*)
             (:right cram-tf:*robot-right-tool-frame*))
           0.0
           transf))

    (print "------------------------------")
    (print "THIS IS THE END TRANSFORM:")
    (print "------------------------------")
    (print end-transf)
    end-transf))
(defmethod get-object-type-to-gripper-pregrasp-transform ((object-type (eql :ba-fork))
                                                          object-name
                                                          arm
                                                          (grasp (eql :human-grasp))
                                                          grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :z-offset *cutlery-pregrasp-z-offset*))
(defmethod get-object-type-to-gripper-lift-transform ((object-type (eql :ba-fork))
                                                      object-name
                                                      arm
                                                      (grasp (eql :human-grasp))
                                                      grasp-transform)
  (get-object-type-to-gripper-pregrasp-transform object-type object-name arm grasp grasp-transform))
