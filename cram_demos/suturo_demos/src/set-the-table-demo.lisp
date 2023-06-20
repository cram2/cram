(in-package :su-demos)

(defun bullet-test()
  (roslisp:start-ros-node "bullet_world")
  (cram-occupancy-grid-costmap::init-occupancy-grid-costmap)
  (cram-bullet-reasoning-belief-state::ros-time-init)
  (cram-location-costmap::location-costmap-vis-init)
  (cram-tf::init-tf)
  (let ((robot-urdf
          (cl-urdf:parse-urdf
           (roslisp:get-param "robot_description"))))
    (prolog:prolog
     `(and (btr:bullet-world ?world)
           (btr:debug-window ?world)
           (cram-robot-interfaces:robot ?robot)
           (assert (btr:object ?world :urdf ?robot ((0 0 0) (0 0 0 1)) :urdf ,robot-urdf))
           ;; (-> (rob-int:robot-joint-states ?robot :arm :left :park ?left-joint-states)
           ;;     (assert (btr:joint-state ?world ?robot ?left-joint-states))
           ;;     (true))
           )))
  (let ((kitchen-urdf 
          (cl-urdf:parse-urdf 
           (roslisp:get-param "kitchen_description"))))
    (prolog:prolog
     `(and (btr:bullet-world ?world)
           (assert (btr:object ?world :urdf :kitchen ((0 0 0) (0 0 0 1))
                                            :urdf ,kitchen-urdf
                                            :collision-group :static-filter
                                            :collision-mask (:default-filter :character-filter)
                                            :compound T)))))

  
  )

;; @author Luca Krohm
;; If called with mode "nav" it will only test the navigation part of the demo
;; If called with mode "perc" it will only test the first perception part of the demo
;; If called with mode "mani" it will only test the first manipulation part of the demo, using fake perception poses
;; If called with mode "perc2" it will only test the second perception part of the demo
;; If called with mode "mani2" it will only test the second manipulation part of the demo, using fake perception poses
;; If called with mode "all" it will test the whole demo
;;
;; IMPORTANT:
;; This parameterization is working, but its WIP. Thinking of writing a macro for it or using another alternative
;; So far it worked in speeding up our testing, since we dont constantly have to add and remove stuff from the
;; code to test specific parts, although this also has its limits, for example we currently cant test perception
;; and manipulation with just one call, without using 'all', which obviously also triggers navigation
(defun set-the-table-demo(mode)
  ;;(with-hsr-process-modules
  ;;(urdf-proj:with-simulated-robot
    (unwind-protect
         (progn
           (roslisp-utilities:startup-ros)

           ;; Calls knowledge to retrieve the percieve pose of the Drawer.
           ;; Moves to the drawer immediatly afterwards.
           (when (or (string-equal mode "all")
                     (string-equal mode "nav"))
             (let* ((urdf (call-knowledge2 "has_urdf_name"
                                           :param-list (list "drawer:drawer:drawer_front_top")
                                           :result 'object))
                    (pose (call-knowledge "object_rel_pose"
                                          :param-list (list urdf "perceive")
                                          :result 'pose))
                    (frame (first pose))
                    (vector (apply #'cl-tf2::make-3d-vector (second pose)))
                    (rotation (apply #'cl-tf2::make-quaternion (third pose))))
                                  
             (move-hsr (cl-tf2::make-pose-stamped frame 0 vector rotation))))

           (when (or (string-equal mode "all")
                     (string-equal mode "perc"))
             
             ;; Detects the drawer handle, the drawer handle is defined as "cup" for testing purposes,
             ;; will be changed in the future. Calls detecting Designator.
             (let* ((?source-object-desig
                      (desig:an object
                                (type :cup)))
                    (?source-perceived-object-desig
                      (exe:perform (desig:an action
                                             (type detecting)
                                             (object ?source-object-desig)))))
              
               ;;Extracts pose from the return value of the detecting Designator.
               (roslisp:with-fields 
                   ((?pose
                     (cram-designators::pose cram-designators:data))) 
                   ?source-perceived-object-desig

                 (when (or (string-equal mode "all")
                           (string-equal mode "mani"))
                 
                   ;;Moves the gripper to the cereal box, implicitly opens the gripper beforehand.
                   (let* ((?grasp-type 0)
                          (?open-drawer 1)
                          (?pose (cond ((string-equal mode "mani") (perceive-handle-closed))
                                         (t ?pose))))
                     (exe:perform (desig:a motion
                                           (type :moving-tcp)
                                           (box-pose ?pose)
                                           (grasp-type ?grasp-type)
                                           (collision-mode :allow-all)))

                     ;;Function call that closes/opens the gripper. We are aware that this should not stay this way. We ran
                     ;;into unexpected error messages when calling the designator opening-gripper/closing-gripper,
                     ;;so we decided to keep this custom call the way it was as there is no point to creating a
                     ;; redundand gripper designator
                     ;;(giskard::call-custom-gripper-action :open-gripper 0)

                     ;;Here we only open the gripper for demo/test purposes, as long as the drawer gets stuck
                     ;;when trying to open it. As soon as that is fixed we can just remove this line
                     ;;(giskard::call-custom-gripper-action :open-gripper 1)

                     ;; Opens the drawer by driving backwards and retracting the arm a little bit
                     (exe:perform (desig:a motion
                                           (type :moving-tcp)
                                           (knob-pose ?pose)
                                           (open-drawer ?open-drawer)
                                           (collision-mode :allow-all))))))))

           (when (or (string-equal mode "all")
                     (string-equal mode "perc2"))

             ;; Detects the drawer handle, the drawer handle is defined as "cup" for testing purposes,
             ;; will be changed in the future. Calls detecting Designator.
             (let* ((?source-object-desig
                      (desig:an object
                                (type :cup)))
                    (?source-perceived-object-desig
                      (exe:perform (desig:an action
                                             (type detecting)
                                             (object ?source-object-desig)))))
              
               ;;Extracts pose from the return value of the detecting Designator.
               (roslisp:with-fields 
                   ((?pose
                     (cram-designators::pose cram-designators:data))) 
                   ?source-perceived-object-desig

                   (when (or (string-equal mode "all")
                             (string-equal mode "perc2")
                             (string-equal mode "mani2"))
                   
                   ;; Opens the drawer by driving forwards and extending the arm a little bit
                     (let* ((?open-drawer 1)
                            (?pose (cond ((string-equal mode "mani2") (perceive-handle-opened))
                                         (t ?pose))))
                       (exe:perform (desig:a motion
                                             (type :moving-tcp)
                                             (knob-pose ?pose)
                                             (open-drawer ?open-drawer)
                                             (collision-mode :allow-all)))))))))
      (roslisp-utilities:shutdown-ros)))
         
