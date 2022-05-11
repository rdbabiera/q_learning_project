#!/usr/bin/env python3

import rospy
import numpy as np
import os

from q_learning_project.srv import StateService, StateServiceResponse

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

# Path of directory to locate where to store q_matrices
qmatrix_path = os.path.dirname(__file__) + "/q_matrix/"


class StateController(object):
    def __init__(self):
        # Intialize Node
        rospy.init_node('q_state_controller')

        # Get QMatrix and Action Matrix
        self.q_matrix = np.loadtxt(qmatrix_path + "q_matrix.csv", delimiter=',')
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # State Variables
        self.recent_state = 0
        self.curr_state = 0

        """
        Sleep for Warmup
        """
        rospy.sleep(3)

    def handle_action_request(self, req):
        action = -1
        prize = 0
        # If Ready, Get First Best Action from Curr State
        if req.status == "Ready":
            state_index = self.curr_state
            # Get Next Action from State, Update Recent State and Curr State
            for i, val in enumerate(self.q_matrix[state_index]):
                if val > prize:
                    prize = val
                    action = i

            # Set Save State, Get Next State
            self.recent_state = self.curr_state
            for j, act in enumerate(self.action_matrix[state_index]):
                if act == action:
                    self.curr_state = j
                    break
        # If Failure, Get First Best Action from Most Recent State
        elif req.status == "Failure":
            state_index = self.recent_state
            for i, val in enumerate(self.q_matrix[state_index]):
                if val > prize:
                    prize = val
                    action = i
                    
            for j, act in enumerate(self.action_matrix[state_index]):
                if act == action:
                    self.curr_state = j
                    break
        print(f"[SERVER] Sending action {action}")
        return StateServiceResponse(action)

    def run(self):
        s = rospy.Service('action_request_service', StateService, self.handle_action_request)
        print("[SERVER] Ready to Handle Requests")
        rospy.spin()

if __name__=="__main__":
    node = StateController()
    node.run()