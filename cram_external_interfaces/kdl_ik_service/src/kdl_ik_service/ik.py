import PyKDL
import rospy
import urdf_parser_py.urdf  # from urdfdom_py
import kdl_parser_py.urdf  # from kdl_parser_py
import tf2_kdl
import math


# ================================= API ===================================================
def calculate_ik(base_link, tip_link, seed_joint_state, goal_transform_geometry_msg):
    """
    Calculates the Inverse Kinematics from base_link to tip_link according to the given
    goal_transform_geometry_msg. The initial joint states would be considered from seed_joint_state.
    Returns the result joint states and the success status.
    base_link eg. - "triangle_base_link" or "calib_left_arm_base_link" or "calib_right_arm_base_link"
    tip_link eg. - "left_arm_7_link" or "right_arm_7_link"
    """
    robot_urdf_string = rospy.get_param('robot_description')
    urdf_obj = urdf_parser_py.urdf.URDF.from_xml_string(robot_urdf_string)
    _, kdl_tree = kdl_parser_py.urdf.treeFromUrdfModel(urdf_obj)
    kdl_chain = kdl_tree.getChain(base_link, tip_link)

    num_joints = kdl_chain.getNrOfJoints()
    print "number of joints: " + str(num_joints)

    # Get Joint limits
    kdl_joint_limits_min, kdl_joint_limits_max = get_kdl_joint_limit_arrays(kdl_chain, urdf_obj)

    fk_solver = PyKDL.ChainFkSolverPos_recursive(kdl_chain)
    velocity_ik = PyKDL.ChainIkSolverVel_pinv(kdl_chain)
    # ik_solver = PyKDL.ChainIkSolverPos_LMA(kdl_chain, 1e-5, 1000, 1e-15)
    ik_solver = PyKDL.ChainIkSolverPos_NR_JL(kdl_chain, kdl_joint_limits_min, kdl_joint_limits_max,
                                             fk_solver, velocity_ik)

    # Getting the goal frame and seed state
    goal_frame_kdl = tf2_kdl.transform_to_kdl(goal_transform_geometry_msg)
    seed_joint_state_kdl = get_kdl_jnt_array_from_list(num_joints, seed_joint_state)

    # Solving IK
    result_joint_state_kdl = solve_ik(ik_solver, num_joints, seed_joint_state_kdl, goal_frame_kdl)

    # check if calculated joint state results in the correct end-effector position using FK
    goal_pose_reached = check_ik_result_using_fk(fk_solver, result_joint_state_kdl, goal_frame_kdl)

    
    if not goal_pose_reached:
        # try with joint seed states as 0
        print "Cannot reach goal using the IK solution with the provided seed state. Trying with zeros"
        result_joint_state_kdl_with_zero_seed = solve_ik(ik_solver, num_joints, PyKDL.JntArray(num_joints), goal_frame_kdl)
        goal_pose_reached = check_ik_result_using_fk(fk_solver, result_joint_state_kdl_with_zero_seed, goal_frame_kdl)
        result_joint_state_kdl = result_joint_state_kdl_with_zero_seed if goal_pose_reached else result_joint_state_kdl 

    # check if calculated joint state is within joint limits
    joints_within_limits = check_result_joints_are_within_limits(num_joints, result_joint_state_kdl,
                                                                 kdl_joint_limits_min, kdl_joint_limits_max)

    print "Result Joint State Within Limits: " + str(joints_within_limits)
    print "Can Reach Goal Pose With Solution: " + str(goal_pose_reached)
    result_joint_state_vector = get_list_from_kdl_jnt_array(num_joints, result_joint_state_kdl)
    goal_pose_reached_successfully = goal_pose_reached and joints_within_limits

    return result_joint_state_vector, goal_pose_reached_successfully


# ================================= Helper ===================================================
def get_joint_names_from_kdl_chain(kdl_chain):
    number_of_segments = int(kdl_chain.getNrOfSegments())
    joint_names = []
    for i in range(0, number_of_segments):
        joint_names.append(kdl_chain.getSegment(i).getJoint().getName())

    return joint_names


def get_joint_limits_from_urdf(joint_name, urdf_obj):
    urdf_joints = urdf_obj.joints
    try:
        [joint_found] = [joint for joint in urdf_joints if joint.name == joint_name]
    except ValueError as e:
        raise ValueError("Error while trying to find joint named: %s in the urdf. Reason: %s" % (joint_name, e))
    if joint_found.joint_type != 'fixed':
        return joint_found.limit.upper, joint_found.limit.lower
    else:
        return None, None


def get_kdl_joint_limit_arrays(kdl_chain, urdf_obj):
    num_joints = kdl_chain.getNrOfJoints()
    kdl_limits_min = PyKDL.JntArray(num_joints)
    kdl_limits_max = PyKDL.JntArray(num_joints)
    index = 0
    for joint_name in get_joint_names_from_kdl_chain(kdl_chain):
        upper_limit, lower_limit = get_joint_limits_from_urdf(joint_name, urdf_obj)
        if not ((upper_limit or lower_limit) is None):
            kdl_limits_max[index] = upper_limit
            kdl_limits_min[index] = lower_limit
            index += 1
    return kdl_limits_min, kdl_limits_max


def get_kinematics_solvers(kdl_chain, kdl_limits_min, kdl_limits_max):
    fk_solver = PyKDL.ChainFkSolverPos_recursive(kdl_chain)
    velocity_ik = PyKDL.ChainIkSolverVel_pinv(kdl_chain)
    # ik_solver = PyKDL.ChainIkSolverPos_LMA(kdl_chain, 1e-5, 1000, 1e-15)
    ik_solver = PyKDL.ChainIkSolverPos_NR_JL(kdl_chain, kdl_limits_min, kdl_limits_max,
                                             fk_solver, velocity_ik)
    return fk_solver, ik_solver


def get_kdl_jnt_array_from_list(num_joints, joint_state_list):
    joint_state_kdl = PyKDL.JntArray(num_joints)
    for index, q in enumerate(joint_state_list):
        joint_state_kdl[index] = q
    return joint_state_kdl


def get_list_from_kdl_jnt_array(num_joints, joint_state_kdl):
    joint_state_vector = []
    for joint in range(0, num_joints):
        joint_state_vector.append(joint_state_kdl[joint])
    return joint_state_vector


def solve_ik(ik_solver, num_joints, seed_joint_state_kdl, goal_frame_kdl):
    result_joint_state_kdl = PyKDL.JntArray(num_joints)
    ik_solver.CartToJnt(seed_joint_state_kdl, goal_frame_kdl, result_joint_state_kdl)
    return result_joint_state_kdl


def check_ik_result_using_fk(fk_solver, result_joint_state_kdl, goal_frame_kdl):
    fk_frame_kdl = PyKDL.Frame()
    fk_solver.JntToCart(result_joint_state_kdl, fk_frame_kdl)

    distance_vec = PyKDL.diff(fk_frame_kdl.p, goal_frame_kdl.p)
    norm = math.sqrt(PyKDL.dot(distance_vec, distance_vec))
    print "distance norm: " + str(norm)

    relative_rotation = fk_frame_kdl.M.Inverse() * goal_frame_kdl.M
    distance_angle, _ = relative_rotation.GetRotAngle()
    print "distance angle: " + str(distance_angle)

    goal_pose_reached = (norm < 0.005) and (abs(distance_angle) < 0.1)
    return goal_pose_reached


def check_result_joints_are_within_limits(num_joints, result_joint_state, joint_limits_min, joint_limits_max):
    for joint in range(0, num_joints):
        if (result_joint_state[joint] > joint_limits_max[joint]) or\
                (result_joint_state[joint] < joint_limits_min[joint]):
            return False
    return True
