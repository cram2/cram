# KDL_IK_SERVICE

Service to solve the Inverse Kinematics of the robot joint states from CRAM.

The service can take in the base link and the tip link of the robotic appendage to be moved, the goal position of the tip link with respect to the base link and the current state of all the non-fixed joints between the base link and tip-link.

### Format:
The service takes in requests in the form of *moveit_msgs/GetPositionIK*. For the full format structure, call the following command from the terminal: 
```rossrv show moveit_msgs/GetPositionIK ```  
While not all the arguments are mandatory, it is essential to provide the ones marked below for the service to work
```
ik_request:
  pose_stamped:
    header:
      frame_id: '<base link name>'
    pose:
      position:
        x: <x offset of the position for the goal w.r.t frame_id>
        y: <y offset of the position for the goal w.r.t frame_id>
        z: <z offset of the position for the goal w.r.t frame_id>
      orientation: 
        x: <x value of the quaternion for the goal w.r.t. frame_id>
        y: <y value of the quaternion for the goal w.r.t. frame_id>
        z: <z value of the quaternion for the goal w.r.t. frame_id>
        w: <w value of the quaternion for the goal w.r.t. frame_id>
  ik_link_name: '<tip link name>'
```
*Note: Replace the values in the angluar brackets(<>) with the actual values for testing*

### Example:
Let's try this using the boxy robot. Bring up the boxy URDF and the IK service using the command  
```roslaunch cram_boxy_assembly_demo sandbox.launch```  
in the terminal.  

We will try to move the left arm including the torso joint. Thus our base link is *base_footprint* and our tip link is *left_arm_7_link*

Our goal pose is :
* position (x: 1.1285, y: 0.0999, z: 1.3212)
* orientation quaternion (x: 1, y: 0, z: 0, w: 0)
in the frame *base_footprint*

Plug in the values as per the format provided above and invoke the service call in a fresh terminal window as shown below: 
``` 
rosservice call /kdl_ik_service/get_ik "
ik_request:
  pose_stamped:
    header:                     
      frame_id: 'base_footprint'
    pose:
      position:
        x: 1.285
        y: 0.0999
        z: 1.3212
      orientation: 
        x: 1
        y: 0
        z: 0
        w: 0                     
  ik_link_name: 'left_arm_7_link'
"
```

A response would be provided in the command line inlcluding the IK solution.

#### Providing the seed state:
Since no joint state is provided for the IK solver in the previous situation, it is assumed that all the joint values are at 0.  

To provide the seed state of the robot joints along with the input request the input command can be modified as below:
```
rosservice call /kdl_ik_service/get_ik "
ik_request:
  robot_state:
    joint_state:
      name: 
        ['triangle_base_joint',
        'left_arm_0_joint', 
        'left_arm_1_joint', 
        'left_arm_2_joint',
        'left_arm_3_joint',
        'left_arm_4_joint',
        'left_arm_5_joint',
        'left_arm_6_joint']
      position:
          [0, -1.858, 0.70751, 0.9614, -2.5922, -2.5922, -1.94065, 2]
  pose_stamped:
    header:
      frame_id: 'base_footprint'
    pose:
      position:
        x: 1.285
        y: 0.0999
        z: 1.3212
      orientation: 
        x: 1
        y: 0
        z: 0
        w: 0
  ik_link_name: 'left_arm_7_link'
"
```
* The values under ```ik_request['robot_state']['joint_state']['position']``` are the values of the __non-fixed joints__ starting from the *base_footprint*. The values are ordered and read sequentially.
* The names under ```ik_request['robot_state']['joint_state']['name']``` are the names of the __non-fixed joints__ starting from *base_footprint* in order, and corresponds to the respective joint values. It is optional and is not used to calculate the IK, but is helpful in organizing the results.

```lisp
Note: The results obtained are sensitive to the input seed state. If you get any invalid result with the existing seed state, try applying 0 as the seed state
```