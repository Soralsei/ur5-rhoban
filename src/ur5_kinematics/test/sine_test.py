from math import sin
import rospy
from actionlib import SimpleActionClient
from ur5_kinematics.msg import URGoToAction, URGoToGoal, URGoToFeedback, URGoToResult
from placo_utils.tf import tf as ptf

import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

# import tf2_geometry_msgs, tf2_ros

import sys

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


if __name__ == '__main__':
    
    def timer_cb(event: rospy.timer.TimerEvent):
        global t, seq
        # t1 = rospy.Time.now()
        T_world_target = ptf.translation_matrix([0.5, 0.3 * np.sin(t), 0.25]) @ ptf.euler_matrix(np.pi, 0, 0)
            
        goal = URGoToGoal()
        goal.target_pose = matrix_to_pose(T_world_target, 'base_link')
        goal.timeout = 2.0
        goal.target_pose.header.seq = seq
        
        client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(2.0))
        
        t += 1 / 100.
        seq += 1
    
    rospy.init_node("kinematics_test", sys.argv)
    
    client = SimpleActionClient('goal_pose', URGoToAction)
    # Waits until the action server has started up and started
    client.wait_for_server()
    
    rospy.loginfo(f'Connected to action server')
    
    timer = rospy.Timer(rospy.Duration(1 / 100.), timer_cb)
    
    t = 0
    seq = 0
    
    rospy.spin()