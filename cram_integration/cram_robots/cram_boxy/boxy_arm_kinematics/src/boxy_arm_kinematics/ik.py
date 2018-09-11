import PyKDL
import rospy
import urdf_parser_py.urdf # from urdfdom_py
import kdl_parser_py.urdf # from kdl_parser_py
import tf2_kdl
import math


def calculate_ik(base_link,tip_link,seed_joint_state, goal_transform_geometry_msg):
    """base_link can be "triangle_base_link" or "calib_left_arm_base_link" or "calib_right_arm_base_link"
    tip_link can be "left_arm_7_link" or "right_arm_7_link"
    joint_state is a vector of 7
    """
    robot_urdf_string = rospy.get_param('robot_description')
    urdf_obj = urdf_parser_py.urdf.URDF.from_xml_string(robot_urdf_string)
    _, kdl_tree = kdl_parser_py.urdf.treeFromUrdfModel(urdf_obj)
    kdl_chain = kdl_tree.getChain(base_link, tip_link)

    num_joints = kdl_chain.getNrOfJoints()
    print "number of joints: " + str(num_joints)


    # calculate joint limits
    # urdf_obj.
    #
    # joints = {}
    # for j in self._baxter.joints:
    #     if j.type != 'fixed':
    #         joints[j.name] = j
    # self.joint_limits_lower = []
    # self.joint_limits_upper = []
    # self.joint_types = []
    # for jnt_name in self._joint_names:
    #     jnt = joints[jnt_name]
    #     if jnt.limit is not None:
    #         self.joint_limits_lower.append(jnt.limit.lower)
    #         self.joint_limits_upper.append(jnt.limit.upper)
    #     else:
    #         self.joint_limits_lower.append(None)
    #         self.joint_limits_upper.append(None)
    #     self.joint_types.append(jnt.type)
    # def replace_none(x, v):
    #     if x is None:
    #         return v
    #     return x
    # self.joint_limits_lower = np.array([replace_none(jl, -np.inf)
    #                                     for jl in self.joint_limits_lower])
    # self.joint_limits_upper = np.array([replace_none(jl, np.inf)
    #                                     for jl in self.joint_limits_upper])
    #     self.joint_types = np.array(self.joint_types)

    joint_limits_left_min = [-2.6179938, -1.8675023, -2.6179938, -1.8675023, -2.6179938, -2.0071287, -2.6179938]
    joint_limits_left_max = [2.6179938, 1.8675023, 2.6179938, 1.8675023, 2.6179938, 2.0071287, 2.6179938]

    kdl_limits_min = PyKDL.JntArray(num_joints)
    for idx,jnt in enumerate(joint_limits_left_min):  kdl_limits_min[idx] = jnt
    kdl_limits_max = PyKDL.JntArray(num_joints)
    for idx,jnt in enumerate(joint_limits_left_max):  kdl_limits_max[idx] = jnt

    fk_solver = PyKDL.ChainFkSolverPos_recursive(kdl_chain)
    velocity_ik = PyKDL.ChainIkSolverVel_pinv(kdl_chain)
    #ik_solver = PyKDL.ChainIkSolverPos_LMA(kdl_chain, 1e-5, 1000, 1e-15)
    ik_solver = PyKDL.ChainIkSolverPos_NR_JL(kdl_chain, kdl_limits_min, kdl_limits_max, fk_solver, velocity_ik)

    goal_frame_kdl = tf2_kdl.transform_to_kdl(goal_transform_geometry_msg)
    seed_joint_state_kdl = PyKDL.JntArray(num_joints)
    for i, q in enumerate(seed_joint_state):
        seed_joint_state_kdl[i] = q

    result_joint_state_kdl = PyKDL.JntArray(num_joints)
    ik_solver.CartToJnt(seed_joint_state_kdl, goal_frame_kdl, result_joint_state_kdl)

    # check if calculated joint state results in the correct end-effector position using FK
    fk_frame_kdl = PyKDL.Frame()
    fk_solver.JntToCart(result_joint_state_kdl, fk_frame_kdl)
    distance_vec = PyKDL.diff(fk_frame_kdl.p, goal_frame_kdl.p)
    norm = math.sqrt(PyKDL.dot(distance_vec, distance_vec))
    print "distance norm: " + str(norm)
    relative_rotation = fk_frame_kdl.M.Inverse() * goal_frame_kdl.M
    distance_angle, _ = relative_rotation.GetRotAngle()
    print "distance angle: " + str(distance_angle)
    goal_pose_reached = (norm < 0.005) and (abs(distance_angle) < 0.1)

    # check if calculated joint state is within joint limits
    result_joint_state_vector = [result_joint_state_kdl[0],
                                 result_joint_state_kdl[1],
                                 result_joint_state_kdl[2],
                                 result_joint_state_kdl[3],
                                 result_joint_state_kdl[4],
                                 result_joint_state_kdl[5],
                                 result_joint_state_kdl[6]]
    for i, q in enumerate(result_joint_state_vector):
        if q > joint_limits_left_max[i] or q < joint_limits_left_min[i]:
            goal_pose_reached = False

    return result_joint_state_vector, goal_pose_reached