#!/usr/bin/env python
import rospy

from abc import abstractmethod

from actionlib import SimpleActionClient
from ur5_kinematics.msg import URGoToAction
from control_msgs.msg import FollowJointTrajectoryAction
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from placo_utils.tf import tf as ptf
import numpy as np

from ur5_kinematics.msg import URGoToGoal


class BaseTest():
    
    def __init__(self, controller_topic: str, kinematics_topic: str, prefix: str = ''):
        self.controller_client = SimpleActionClient(controller_topic, FollowJointTrajectoryAction)
        self.kinematics_client = SimpleActionClient(kinematics_topic, URGoToAction)
        self.base_frame = f'{prefix}base_link'
        
        self.controller_client.wait_for_server()
        rospy.loginfo(f'Connected to controller action server')
        self.kinematics_client.wait_for_server()
        rospy.loginfo(f'Connected to kinematics action server')
        
    
    def init_goal(self, target) -> URGoToGoal:
        T_world_target = ptf.translation_matrix(target) @ ptf.euler_matrix(np.pi, 0, 0)
            
        goal = URGoToGoal()
        goal.target_pose = BaseTest.matrix_to_pose(T_world_target, self.base_frame)
        goal.timeout = 2.0
        # goal.duration = rospy.Duration(2.0)
        goal.duration = rospy.Duration(self.segment_duration)
        goal.target_pose.header.seq = self.seq
    
        return goal
    

    @staticmethod
    def matrix_to_pose(matrix: np.ndarray, frame: str = 'base_link') -> np.ndarray:
        #Transform point into arm base frame
        pos = ptf.translation_from_matrix(matrix)
        rot = ptf.quaternion_from_matrix(matrix)
        
        header = Header()
        header.frame_id = frame
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]
        pose.pose.orientation.w = rot[0]
        pose.pose.orientation.x = rot[1]
        pose.pose.orientation.y = rot[2]
        pose.pose.orientation.z = rot[3]
        
        return pose
    
    
    @abstractmethod
    def update(self):
        pass        