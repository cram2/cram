(in-package :kvr)

;; orientation source: cram_knowrob_pick_place
;; object knowledge ------------------------------------------------------------
;; TODO compare this to other packadges, to make sure it is enough.
;; TODO maybe move it to items.lisp? What is other cad specific knowledge?
(prolog:def-fact-group kvr-object-knowledge (object-rotationally-symmetric
                                             orientation-matters
                                             obj-int:object-type-grasp)
  
  (prolog:<- (object-rotationally-symmetric ?object-type)
    (member ?object-type (:cup-eco-orange :weide-milch-small)))

  (prolog:<- (orientation-matters ?object-type)
    (member ?object-type (:fork-blue-plastic
                          :spoon-blue-plastic
                          :edeka-red-bowl
                          :koelln-muesli-knusper-honig-nuss)))
            
  (prolog:<- (obj-int:object-type-grasp :fork-blue-plastic :human-grasp))
  (prolog:<- (obj-int:object-type-grasp :fork-blue-plastic :top))

  (prolog:<- (obj-int:object-type-grasp :spoon-blue-plastic :human-grasp))
  (prolog:<- (obj-int:object-type-grasp :spoon-blue-plastic :top))
  
  (prolog:<- (obj-int:object-type-grasp :edeka-red-bowl :human-grasp))
  (prolog:<- (obj-int:object-type-grasp :edeka-red-bowl :top))
  
  (prolog:<- (obj-int:object-type-grasp :cup-eco-orange :human-grasp))
  (prolog:<- (obj-int:object-type-grasp :cup-eco-orange :back))
  (prolog:<- (obj-int:object-type-grasp :cup-eco-orange :front))
  (prolog:<- (obj-int:object-type-grasp :cup-eco-orange :left-side))
  (prolog:<- (obj-int:object-type-grasp :cup-eco-orange :right-side))

  (prolog:<- (obj-int:object-type-grasp :weide-milch-small :human-grasp))
  (prolog:<- (obj-int:object-type-grasp :weide-milch-small :back))
  (prolog:<- (obj-int:object-type-grasp :weide-milch-small :left-side))
  (prolog:<- (obj-int:object-type-grasp :weide-milch-small :right-side))
  (prolog:<- (obj-int:object-type-grasp :weide-milch-small :front))
  
  (prolog:<- (obj-int:object-type-grasp :koelln-muesli-knusper-honig-nuss :human-grasp))
  (prolog:<- (obj-int:object-type-grasp :koelln-muesli-knusper-honig-nuss :back))
  (prolog:<- (obj-int:object-type-grasp :koelln-muesli-knusper-honig-nuss :front)))



;; grasps ----------------------------------------------------------------------
;;  General Grasp
;; TODO: generalize this grasp. Type could and should be derived from the VR data
;; Name of type string or use filter?
(defmethod get-object-type-to-gripper-transform (object-type
                                                 object-name
                                                 arm
                                                 (grasp (eql :human-grasp)))
  (let* (transf
         end-transf)

    ;; transf. from Map to Obj?
    (setq transf
          (cl-tf:transform*
           (cl-tf:transform-inv
            ;; TODO object-name or type?
            (get-object-location-at-start-by-object-type object-name))
           (get-hand-location-at-start-by-object-type object-name)
           (human-to-robot-hand-transform)))
    (setf end-transf
          (cl-tf:transform->transform-stamped
           (roslisp-utilities:rosify-underscores-lisp-name object-name)
           (ecase arm
             (:left cram-tf:*robot-left-tool-frame*)
             (:right cram-tf:*robot-right-tool-frame*))
           0.0
           transf))
    end-transf))

(defmethod get-object-type-to-gripper-pregrasp-transform (object-type
                                                          object-name
                                                          arm
                                                          (grasp (eql :human-grasp))
                                                          grasp-transform)
  
  (cram-tf:translate-transform-stamped
   grasp-transform
   :x-offset (- cram-knowrob-pick-place::*cereal-pregrasp-xy-offset*)
   :z-offset cram-knowrob-pick-place::*lift-z-offset*))

(defmethod get-object-type-to-gripper-2nd-pregrasp-transform (object-type
                                                              object-name
                                                              arm
                                                              (grasp (eql :human-grasp))
                                                              grasp-transform)
  (cram-tf:translate-transform-stamped
   grasp-transform
   :x-offset (- cram-knowrob-pick-place::*cereal-pregrasp-xy-offset*)))

