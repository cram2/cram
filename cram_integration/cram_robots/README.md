
# Using the HSR in Bullet World (Cram) Tutorial



## Robot-URDF

The robot urdf of the hsr contains a  gravel(`) symbol. This causes some errors when reading the urdf from the param-server inside of roslisp. 

We also included a **gripper-tool-frame** for the urdf manualy, since toyota did not defined any.

For the **base_footprint** we are added a small 0.1x0.1x0.1 box, to check for enviroment collisions.

Lasty we pimped the robot with soem colors inside the urdf.

## Arm-Kinematics - IK-Solver
The  arm kinematics are resolved by an ik solver. The ik solver tries multiple ways for the joints to be able to solve the goal position of the arm.    

The hsr as only 5 joints on the arm. Therefor he has a very limited range and is not able to grasp all objects .

The ik solver is very simple solution written in python. This implementation is made for the bulletworld only. Within the rapid fire projection we just want to quickly check if its possible at all for the robots to grasp the objects.



## Robot description 
 Similar to cram_pr2_description or cram_boxy_description, it should contain Prolog queries describing the robot semantically. You need to decide how you define each part of the robot for example Boxy does have a own neck.lisp file in the package. The HSR is similar to the PR2, hence the description package contains arm.lisp and knowledge.lisp.

The queries are implementations of the interfaces defined in cram_robot_interfaces. It is important that the joints and links are correctly sorted. 


### Arms.lisp
    #(in-package :cram-hsrb-description)

    (defparameter *tcp-in-ee-pose*
      (cl-transforms:make-pose
       (cl-transforms:make-3d-vector 0.0d0 0.0d0 0.23090001999999998d0)
       (cl-transforms:make-quaternion 0.0d0
                                  0.0d0
                                  0.7071067811865475d0
                                  0.7071067811865476d0)))

    (defparameter *standard-to-hsrb-gripper-transform*
      (cl-transforms-stamped:make-identity-transform))


We need to describe the tool-center-point pose within the ee pose for future calculation of the gripper. 

    (def-fact-group hsrb-arm-facts (arm
                                arm-joints arm-links
                                hand-links hand-link hand-finger-link
                                gripper-joint
                                gripper-meter-to-joint-multiplier
                                gripper-minimal-position
                                gripper-convergence-delta
                                standard<-particular-gripper-transform
                                end-effector-link
                                robot-tool-frame
                                tcp-in-ee-pose
                                robot-joint-states)

      (<- (arm :hsrb :left))

      (<- (arm-joints :hsrb :left ("arm_flex_joint"
                                  "arm_roll_joint"
                                  "wrist_flex_joint"
                                  "wrist_roll_joint")))

      (<- (arm-links :hsrb :left ("arm_flex_link"
                                 "arm_roll_link"
                                 "wrist_flex_link"
                                 "wrist_roll_link")))

      (<- (hand-links :hsrb :left ("hand_l_distal_link"
                                   "hand_l_spring_proximal_link"
                                   "hand_palm_link"
                                   "hand_r_distal_link"
                                   "hand_r_spring_proximal_link")))

      (<- (hand-link :hsrb :left ?link)
        (bound ?link)
        (lisp-fun search "hand" ?link ?pos)
        (lisp-pred identity ?pos))

      (<- (hand-finger-link :hsrb ?arm ?link)
        (bound ?link)
        (hand-link :hsrb ?arm ?link)
        (lisp-fun search "palm" ?link ?pos)
        (not (lisp-pred identity ?pos)))

      (<- (gripper-joint :hsrb :left "hand_motor_joint"))

      (<- (gripper-meter-to-joint-multiplier :hsrb 1.0))
      (<- (gripper-minimal-position :hsrb ?_ 0.0))
      (<- (gripper-convergence-delta :hsrb ?_ 0.001))

      (<- (standard<-particular-gripper-transform :hsrb ?transform)
        (symbol-value *standard-to-hsrb-gripper-transform* ?transform))

      (<- (end-effector-link :hsrb :left "wrist_roll_link"))

      (<- (robot-tool-frame :hsrb :left "gripper_tool_frame"))

      (<- (tcp-in-ee-pose :hsrb ?pose)
        (symbol-value *tcp-in-ee-pose* ?pose))
        
All the links and frames defined in prolog way. The hsr only has one arm..i called it left arm bc reasons.        

      (<- (robot-joint-states :hsrb :arm :left :carry
                              (("arm_flex_joint" 0)
                               ("arm_roll_joint" 1.5)
                               ("wrist_flex_joint" -1.85)
                               ("wrist_roll_joint" 0))))
      (<- (robot-joint-states :hsrb :arm :left :park ?joint-states)
        (robot-joint-states :hsrb :arm :left :carry ?joint-states)))

Defining a standart carry pose here, since we will need this pose often in the future and then we can just say robot= carry pose.

### general_knowledge.lisp

    (in-package :hsrb-descr)

    (defparameter *forward-looking-position-in-base-frame*
      (cl-transforms:make-3d-vector 10.0 0.0 1.5))

    (def-fact-group hsrb-metadata (robot-odom-frame
                                   robot-base-frame robot-base-link
                                   robot-torso-link-joint
                                   arm
                                   camera-frame
                                   camera-horizontal-angle camera-vertical-angle
                                   robot-neck-links robot-neck-joints
                                   robot-joint-states robot-pose)

      (<- (robot-odom-frame :hsrb "odom"))
      (<- (robot-base-frame :hsrb "base_footprint"))
      (<- (robot-base-link :hsrb "base_link"))
      (<- (robot-torso-link-joint :hsrb "arm_lift_link" "arm_lift_joint"))

      (<- (camera-frame :hsrb "head_center_camera_frame"))
      (<- (camera-frame :hsrb "head_rgbd_sensor_link"))

      ;; These are values taken from the Kinect's wikipedia page for the 360 variant TODO do this for the hsr?
      (<- (camera-horizontal-angle :hsrb 0.99483))
      (<- (camera-vertical-angle :hsrb 0.75049))

      (<- (robot-neck-links :hsrb "head_pan_link" "head_tilt_link"))
      (<- (robot-neck-joints :hsrb "head_pan_joint" "head_tilt_joint"))

      (<- (robot-joint-states :hsrb :neck ?_ :forward ((?pan_joint 0.0)         (?tilt_joint 0.0)))
        (robot-neck-joints :hsrb ?pan_joint ?tilt_joint))

      (<- (robot-pose :hsrb :neck ?_ :forward ?pose-stamped)
            (robot-base-frame :hsrb ?base-frame)
            (lisp-fun cl-transforms:make-identity-rotation ?identity-quaternion)
            (symbol-value *forward-looking-position-in-base-frame* ?forward-point)
        (lisp-fun cl-transforms-stamped:make-pose-stamped
                  ?base-frame 0.0 ?forward-point ?identity-quaternion
                  ?pose-stamped)))
    
Defining all the links and frames of the robot that are not part of the arm.    
    
    
    (def-fact-group location-costmap-metadata (costmap:costmap-padding
                                               costmap:costmap-manipulation-padding
                                               costmap:costmap-in-reach-distance
                                               costmap:costmap-reach-minimal-distance
                                               costmap:orientation-samples
                                               costmap:orientation-sample-step
                                               costmap:visibility-costmap-size)
      (<- (costmap:costmap-padding :hsrb 0.2))
      (<- (costmap:costmap-manipulation-padding :hsrb 0.3))
      (<- (costmap:costmap-in-reach-distance :hsrb 0.5))
      (<- (costmap:costmap-reach-minimal-distance :hsrb 0.2))
      (<- (costmap:orientation-samples :hsrb 3))
      (<- (costmap:orientation-sample-step :hsrb 0.3))
      (<- (costmap:visibility-costmap-size :hsrb 2)))
      
      
For more information u can visit [new-robots-in-cram-tutorial](http://cram-system.org/tutorials/advanced/new-robot) it needs to be updated tho.



## Cram_hsrb_pick_demo

Within this demo you can see a very small example of different methods and designators.

    ;;roslaunch cram_projection_demos household_hsrb.launch 
    [...]

        (btr-utils:park-robot)
        (let* ((?nav-pose nav-pose)
               (?object-pose
                 (cl-transforms-stamped:pose->pose-stamped
                  "map" 0 (btr:object-pose object-name))))
          ;; perform a action type going to first pose
          (exe:perform
           (desig:an action
                     (type going)
                     (target (desig:a location (pose ?nav-pose)))))

          ;;perform a action type looking towards the object
          (exe:perform
           (desig:an action
                     (type looking)
                     (target (desig:a location (pose ?object-pose))))))

        (let* ((?object-desig
                 (exe:perform (desig:a motion
                                       (type detecting)
                                       (object (desig:an object
                                                         (type ?object-type))))))
               (?grasp (caadar nav-pose)))

          ;; perform a action type picking-up with given grasp and object
          (exe:perform (desig:an action
                                 (type picking-up)
                                 (arm :left)
                                 (grasp ?grasp)
                                 (object ?object-desig))))))))



*To start the demo you will need to:*

`roslaunch cram_hsrb_pick_demo sandbox.launch`

Load the package **cram_hsrb_pick_demo**

You can start the demo with: `(demo::spawn-pickup-cylinder-air)`
