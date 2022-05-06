#!/usr/bin/env python3

import rospy
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math
# import the custom message
from lab_f_traffic_bot.msg import Traffic

class Robot(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('turtlebot3_traffic_controller')

        # Traffic status subscriber
        rospy.Subscriber("/traffic_status", Traffic, self.traffic_dir_received)

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        self.move_group_arm.go([0,0,0,0], wait=True)
        print("ready")

    def traffic_dir_received(self, data: Traffic):
        # TODO: Write code on what the turtlebot should do upon receiving a
        # traffic msg.

        # Tips:
        # arm_joint_goal is a list of 4 radian values, 1 for each joint
        # for instance,
        #           arm_joint_goal = [0.0,0.0,0.0,0.0]
        #           arm_joint_goal = [0.0,
        #               math.radians(5.0),
        #               math.radians(10.0),
        #               math.radians(-20.0)]
    

        # wait=True ensures that the movement is synchronous
        # arm_joint_goal = [0.0, 0.0, 0.0, 0.0]
        arm_joint_goal = [math.radians(data.joints[0]),
                          math.radians(data.joints[1]),
                          math.radians(data.joints[2]),
                          math.radians(data.joints[3]),]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group_arm.stop()
        

        # gripper_joint_goal is a list of 2 radian values, 1 for the left gripper and 1 for the right gripper
        # for instance,
        #           gripper_joint_goal = [-0.009,0.0009]
        #           gripper_joint_goal = [0.0, 0.0]
        gripper_joint_goal = [math.radians(data.grippers[0]),
                              math.radians(data.grippers[1])]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    robot = Robot()
    robot.run()
