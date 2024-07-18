import rospy

from itertools import cycle
from collections.abc import Iterable

from ur5_kinematics.testing import BaseTest

from ur5_kinematics.msg import URGoToResult
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryResult

class PathTest(BaseTest):
    def __init__(self, controller_topic: str, kinematics_topic:str, path: Iterable, prefix:str = '', loop: bool = True, segment_duration: float = 0.0):
        super(PathTest, self).__init__(controller_topic, kinematics_topic, prefix)
        self.path = path
        self.seq = 0
        self.segment_duration = segment_duration
        if loop:
            self.path = cycle(path)
        else:
            self.path = iter(path)

            
    def update(self):
        target = next(self.path)
        goal = self.init_goal(target)
        self.kinematics_client.send_goal_and_wait(goal)
        
        result: URGoToResult = self.kinematics_client.get_result()
        if result.state == URGoToResult.SUCCEEDED:
            traj_goal = FollowJointTrajectoryGoal()
            traj_goal.trajectory = result.trajectory
            
            rospy.loginfo(f'Got trajectory, executing...')
            self.controller_client.send_goal_and_wait(traj_goal)
            res: FollowJointTrajectoryResult = self.controller_client.get_result()

            if res.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
                rospy.loginfo(f'Trajectory execution success')
        else:
            raise RuntimeError('Solver failed')
        self.seq += 1