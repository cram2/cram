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

    fk_solver = PyKDL.ChainFkSolverPos_recursive(kdl_chain)
    velocity_ik = PyKDL.ChainIkSolverVel_pinv(kdl_chain)
    ik_solver = PyKDL.ChainIkSolverPos_LMA(kdl_chain, 1e-5, 1000, 1e-15)

    goal_frame_kdl = tf2_kdl.transform_to_kdl(goal_transform_geometry_msg)
    seed_joint_state_kdl = PyKDL.JntArray(num_joints)
    for i, q in enumerate(seed_joint_state):
        seed_joint_state_kdl[i] = q

    result_joint_state_kdl = PyKDL.JntArray(num_joints)
    ik_solver.CartToJnt(seed_joint_state_kdl, goal_frame_kdl, result_joint_state_kdl)

    result_joint_state_vector = [result_joint_state_kdl[0],
                                 result_joint_state_kdl[1],
                                 result_joint_state_kdl[2],
                                 result_joint_state_kdl[3],
                                 result_joint_state_kdl[4],
                                 result_joint_state_kdl[5],
                                 result_joint_state_kdl[6]]

    fk_frame_kdl = PyKDL.Frame()
    fk_solver.JntToCart(result_joint_state_kdl, fk_frame_kdl)
    distance_vec = PyKDL.diff(fk_frame_kdl.p, goal_frame_kdl.p)
    norm = math.sqrt(PyKDL.dot(distance_vec, distance_vec))
    print norm

    return result_joint_state_vector, (norm < 0.01)