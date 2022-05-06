#!/usr/bin/env python3

import rospy
import numpy as np
import moveit_commander
import math


from q_learning_project.msg import Control
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from q_learning_project.srv import StateService, StateServiceResponse

class Arm(object):


    def __init__(self):


        """
        Setup Arm Control
        """
        # initialize this node
        rospy.init_node('turtlebot3_arm_controller')

        # Traffic status subscriber
        #rospy.Subscriber("/traffic_status", Control, self.traffic_dir_received)

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        self.move_group_arm.go([0,0,0,0], wait=True)
        print("ready")

              

    def move_arm(self, joints):
        
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
        arm_joint_goal = [math.radians(joints[0]),
                          math.radians(joints[1]),
                          math.radians(joints[2]),
                          math.radians(joints[3]),]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group_arm.stop()
              

    
    def open_grip():

        # gripper_joint_goal is a list of 2 radian values, 1 for the left gripper and 1 for the right gripper
        # for instance,
        #           gripper_joint_goal = [-0.009,0.0009]
        #           gripper_joint_goal = [0.0, 0.0]
        gripper_joint_goal = [-0.009,0.0009]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

        

    def close_grip():

        # gripper_joint_goal is a list of 2 radian values, 1 for the left gripper and 1 for the right gripper
        # for instance,
        #           gripper_joint_goal = [-0.009,0.0009]
        #           gripper_joint_goal = [0.0, 0.0]
        gripper_joint_goal = [0.0, 0.0]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

        




class Robot(object):

    def __init__(self):

        """
        Setup open cv image recognition and RGB Camera hardware
        """

        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # initalize the debugging window
        cv2.namedWindow("window", 1)

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('raspicam_node/image', Image, self.image_callback)
        
        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.movement = Twist()
        

        """
        Initialize Arm object
        """

        self.arm = Arm()

        """
        Set Up Action Array, Colors to HSV
        """""""""
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))
        self.color_codes = {}
        self.color_codes["pink"] = {'high': [165, 255, 217], 'low': [161, 171, 161]}
        self.color_codes["green"] = {'high': [34, 255, 230], 'low': [31, 192, 137]}
        self.color_codes["blue"] = {'high': [97, 113, 232], 'low': [99, 94, 122]}

        """
        Sleep for Warmup
        """
        
        rospy.sleep(5)

    def get_action(self):
        rospy.wait_for_service('action_request_service')
        try:
            receive_action = rospy.ServiceProxy('action_request_service', StateService)
            response = receive_action("Ready")
            if response.action == -1:
                print("There's Nothing Left to Do, Shutting Down")
                exit(1)
            self.perform_action(reponse.action)
        except rospy.ServiceException as e::
            print("Service Call Failed: %s"%e)
        return
    
    def perform_action(self, action):
        ### Preliminary Steps


        ### Main Steps
        # 1. Go To Object
        # 2. Pickup Object
        pickup()
        #--- 2.1 Verify Object Picked Up
        # 3. Locate Tag
        # 4. Goto Tag
        # 5. Drop Object
        #--- 5.1 Verify Dropoff
        # 6. Back Away from Object
        

    def pickup():

        arm_straight_goal = [0, 0, 0, 0]

        arm_lift_goal = [81, 45, 39, 58]

        
        self.movement.linear.x = 0.01
        self.movement.angular.z = 0

        
        self.arm.move_arm(arm_straight_goal)
        self.arm.open_grip()

        self.move_pub.publish(movement)
        rospy.sleep(0.2)

        self.arm.close_grip()


        self.arm.move_arm(arm_lift_goal)



    def run(self):
        self.get_action()
        rospy.spin()

if __name__ == "__main__":
    robot = Robot()
    robot.run()

