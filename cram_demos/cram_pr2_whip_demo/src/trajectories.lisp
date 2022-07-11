(in-package :cram-manipulation-interfaces)

;pre-grasp - need to make z into dynamic offset depending on tool
(defmethod get-object-type-robot-frame-whisk-pre-grasp-approach-transform
    (object-type
     arm)
     (eql object-type :big-bowl)
    (eql grasp :center) 
  '((0.0 0.0 0.6)(0 0.707 0 0.707)))
  
;grasp-need to make z into dynamic offset depending on tool- will be done in trajectories this method should be in household at a later time
(defmethod get-object-type-robot-frame-whisk-grasp-transform
    (object-type
     arm)
     (eql object-type :big-bowl)
    (eql grasp :center) 
  '((0.0 0.0 0.3)(0 0.707 0 0.707)))

(defun translate-pose-in-base (bTg &key (x-offset 0.0) (y-offset 0.0) (z-offset 0.0))
  (cram-tf:translate-transform-stamped bTg
                                       :x-offset x-offset
                                       :y-offset y-offset
                                       :z-offset z-offset))
                                    
                                    ;heustistics    
(defun man-int:get-action-trajectory (action-type
                                                          arm
                                                          grasp
                                                          objects-acted-on
                                      
                                                          &key )

(if (eql action-type :mixing)

(let* ((object
           (car objects-acted-on))
         (object-name
           (desig:desig-prop-value object :name))
         (btr-object
           (btr:object btr:*current-bullet-world* object-name))
         (object-type
           (desig:desig-prop-value object :type))
         (bTo
           (man-int:get-object-old-transform object))
         (oTb
           (cram-tf:transform-stamped-inv bTo))
         (bTb-lifts
           (man-int:get-object-type-wrt-base-frame-lift-transforms
            object-type arm grasp location))
         (oTg-std 
           (man-int:get-object-type-to-gripper-transform
            object-type object-name arm grasp))
            
            ;rotation des gripper important -in 
         (bTb-offset(get-object-type-robot-frame-whisk-pre-grasp-approach-transform
                   object-type arm grasp))   
         )
;stuff
    (flet ((get-base-to-gripper-transform-for-whipping (bTb-offset)
             (cl-tf:make-transform-stamped
              (cl-tf:frame-id bTo)
              (if (eql arm :right)
                  "r_gripper_tool_frame"
                  "l_gripper_tool_frame")
              0.0
              (cl-tf:v+
               (cl-tf:translation bTo)
               (cl-tf:translation bTb-offset))
              (cl-tf:rotation bTb-offset))))
      
      (mapcar (lambda (label transforms)
                (man-int:make-traj-segment
                 :label label
                 :poses 
                 (if (eq label :whip-approach)
                    
                 ;gotta infer missing gasp info :center
		 ;whip approach using pouring-> now pre grasp to grasp position
                     (calculate-init-whipping-trajectory-in-map btr-object arm grasp)
                                        
                     (mapcar 
                      (alexandria:curry #'man-int:calculate-gripper-pose-in-map bTo arm)
                      transforms))
                  ;if eq label start mix - rim of bowl has offset for biggest circle    
                      
                      ))
              '(:reaching
                :grasping
                :whip-approach
                ;:start-mix
                ; :mix
                ;:end-mix
                )
              `(,(man-int:get-object-type-to-gripper-pregrasp-transforms
                  object-type object-name arm grasp location oTg-std)
                (,oTg-std)
                (,bTb-offset)
                   ;wip mixing
                ))))))
 
 ;WIP                 
(defun calculate-init-whipping-trajectory-in-map (object arm bTg)
  (let* ((mTb
           (cram-tf:pose->transform-stamped
            cram-tf:*fixed-frame*
            cram-tf:*robot-base-frame*
            0.0
            (btr:pose (btr:get-robot-object)))))
    (mapcar (lambda (bTg-pose)
               (cl-tf:ensure-pose-stamped
                (cram-tf:apply-transform 
                 mTb
                 bTg-pose)))
             (calculate-whipping-trajectory object arm bTg))))
 
 ;(defun calculate-whipping-trajectory ())
 
