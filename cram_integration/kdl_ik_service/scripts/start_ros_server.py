#!/usr/bin/env python

import rospy
import moveit_msgs.srv
import geometry_msgs.msg
import kdl_ik_service.ik


def callback(request):
    # print "Got request %s"%(request)
    debug_mode = rospy.get_param("ik_debug", False)
    ik_logger = Logger(debug_mode)

    response = moveit_msgs.srv.GetPositionIKResponse()

    end_effector_link = request.ik_request.ik_link_name
    ik_logger.log(end_effector_link)

    pose_stamped = request.ik_request.pose_stamped
    transform_stamped = geometry_msgs.msg.TransformStamped
    transform_stamped.header = pose_stamped.header
    transform_stamped.child_frame_id = end_effector_link
    transform_stamped.transform = geometry_msgs.msg.Transform
    transform_stamped.transform.translation = geometry_msgs.msg.Vector3
    transform_stamped.transform.translation.x = pose_stamped.pose.position.x
    transform_stamped.transform.translation.y = pose_stamped.pose.position.y
    transform_stamped.transform.translation.z = pose_stamped.pose.position.z
    transform_stamped.transform.rotation = pose_stamped.pose.orientation
    ik_logger.log(transform_stamped)

    base_link = transform_stamped.header.frame_id
    ik_logger.log(base_link)

    joint_state = request.ik_request.robot_state.joint_state
    ik_logger.log(joint_state)

    timeout = request.ik_request.timeout
    ik_logger.log(timeout)

    response.solution.joint_state = joint_state
    new_joint_state_vector, success = kdl_ik_service.ik.calculate_ik(base_link, end_effector_link, joint_state.position, transform_stamped, ik_logger.log)
    ik_logger.log(new_joint_state_vector)
    response.solution.joint_state.position = new_joint_state_vector

    if success:
        response.error_code.val = response.error_code.SUCCESS
    else:
        response.error_code.val = response.error_code.NO_IK_SOLUTION
    ik_logger.log(response)
    return response


def server_main():
    rospy.init_node('kdl_ik_service')
    server = rospy.Service('~get_ik', moveit_msgs.srv.GetPositionIK, callback)
    print "IK server ready."
    rospy.spin()


class Logger(object):
    def __init__(self, display_output=False):
        self.display_output = display_output

    def log(self, line):
        if self.display_output:
            print line


if __name__ == "__main__":
    server_main()
