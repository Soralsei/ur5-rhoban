from math import sin
import rospy
from itertools import cycle
from actionlib import SimpleActionClient
from ur5_kinematics.msg import URGoToAction, URGoToGoal, URGoToResult
from control_msgs.msg import FollowJointTrajectoryAction,  FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectory
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
    rospy.init_node("kinematics_test", sys.argv)
    points = [
        [0.2, -0.15, 0.35],
        [0.2, 0.15, 0.35],
        [0.5, 0.15, 0.35],
        [0.5, -0.15, 0.35]
    ]
    path = cycle(points)
    
    controller_client = SimpleActionClient('/pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client = SimpleActionClient('goal_pose', URGoToAction)
    trajectory_pub = rospy.Publisher('ik/trajectory', JointTrajectory, queue_size=10)
    # Waits until the action server has started up and started
    client.wait_for_server()
    controller_client.wait_for_server()
    
    rospy.loginfo(f'Connected to action server')
    
    freq = rospy.Rate(0.5)
    t = np.pi / 2.
    seq = 0
    while True:
        target = next(path)
        T_world_target = ptf.translation_matrix(target) @ ptf.euler_matrix(np.pi, 0, 0)
            
        goal = URGoToGoal()
        goal.target_pose = matrix_to_pose(T_world_target, 'base_link')
        goal.timeout = 2.0
        # goal.duration = rospy.Duration(2.0)
        goal.duration = rospy.Duration(0.)
        goal.target_pose.header.seq = seq
        
        rospy.loginfo(f'Generating trajectory to reach point {target}')
        client.send_goal_and_wait(goal)
        result: URGoToResult = client.get_result()
        
        if result.state == URGoToResult.SUCCEEDED:
            traj_goal = FollowJointTrajectoryGoal()
            traj_goal.trajectory = result.trajectory
            # traj_goal.trajectory.header.stamp = rospy.Time.now()
            
            rospy.loginfo(f'Got trajectory, executing...')
            controller_client.send_goal_and_wait(traj_goal)
            res: FollowJointTrajectoryResult = controller_client.get_result()
            if res.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
                rospy.loginfo(f'Trajectory execution success')
            
        t += np.pi
        seq += 1
        freq.sleep()