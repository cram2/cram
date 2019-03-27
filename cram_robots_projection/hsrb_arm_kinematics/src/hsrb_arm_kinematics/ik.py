import PyKDL
import rospy
import urdf_parser_py.urdf # from urdfdom_py
import kdl_parser_py.urdf # from kdl_parser_py
import tf2_kdl
import math


def calculate_ik(base_link,tip_link,seed_joint_state, goal_transform_geometry_msg):
    robot_urdf_string = rospy.get_param('robot_description')
    urdf_obj = urdf_parser_py.urdf.URDF.from_xml_string(robot_urdf_string)
    _, kdl_tree = kdl_parser_py.urdf.treeFromUrdfModel(urdf_obj)
    kdl_chain = kdl_tree.getChain(base_link, tip_link)
    num_joints = kdl_chain.getNrOfJoints()
    print "number of joints: " + str(num_joints)



    joint_limits_left_min = [-2.62, -2.09, -1.92, -1.92]
    joint_limits_left_max = [ 0.00, 3.84, 1.22, 3.67]


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
                                 result_joint_state_kdl[3]]
    for i, q in enumerate(result_joint_state_vector):
        if q > joint_limits_left_max[i] or q < joint_limits_left_min[i]:
            goal_pose_reached = False
    print "ende der funktion"
    print str(result_joint_state_vector)
    print str(goal_pose_reached)

    return result_joint_state_vector, goal_pose_reached
