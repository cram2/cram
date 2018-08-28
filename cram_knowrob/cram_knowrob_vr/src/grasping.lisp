(in-package :kvr)


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

