#!/usr/bin/env python

import rospy
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

GOAL_POINTS = [(-2.58, -0.19, 0),
               (4.83, -5.80, 0),
               (4.73, 1.90, 0),
               (0.0, 0.0, 0.0)]

TIMEOUT = 120

def main():
    for x, y, yaw in GOAL_POINTS:
        rospy.loginfo("move to x={}, y={}, z={}".format(x, y, yaw))
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.header.stamp = rospy.Time.now()
        goal_msg.target_pose.pose.position.x = x
        goal_msg.target_pose.pose.position.y = y

        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal_msg.target_pose.pose.orientation.x = q[0]
        goal_msg.target_pose.pose.orientation.y = q[1]
        goal_msg.target_pose.pose.orientation.z = q[2]
        goal_msg.target_pose.pose.orientation.w = q[3]

        base_ac.send_goal(goal_msg)
        succeeded = base_ac.wait_for_result(rospy.Duration(TIMEOUT))
        if succeeded:
            rospy.loginfo('reached to goal position')
        else:
            rospy.logerr('failed to reach goal')


if __name__ == "__main__":
    rospy.init_node("navigation_tutorial")
    base_ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo('waitig for move_base action server')
    base_ac.wait_for_server(rospy.Duration(2))

    try:
        main()

    except:
        import traceback
        traceback.print_exc()
