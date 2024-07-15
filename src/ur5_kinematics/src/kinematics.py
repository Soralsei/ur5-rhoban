# TODO: Implement better trajectory duration handling (trajectory duration dichotomic search ?)
# TODO: Verify check_acceleration method
# TODO: Implement joint mask/unmask services
import rospy

import numpy as np
from collections.abc import Iterable
import placo
from placo_utils.visualization import robot_viz, frame_viz, robot_frame_viz
from placo_utils.tf import tf as ptf

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TransformStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

from ur5_kinematics.msg import URGoToAction, URGoToFeedback, URGoToResult, URGoToGoal
from ur5_kinematics.srv import ListJoints, ListJointsResponse, SetActiveJoints,\
    SetActiveJointsResponse, UnmaskJoints, MaskJoints, \
    UnmaskJointsResponse, MaskJointsResponse

import actionlib
import tf2_ros as tf2
import tf2_geometry_msgs
import rospkg

import re, sys, time
from threading import Lock, Condition


UNMASKED_JOINT_NAMES=[
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]


def parse_ros_packages(urdf: str):
    if not urdf:
        raise TypeError(f"parse_ros_packages: URDF missing")
    
    rospack = rospkg.RosPack()
    
    packages = re.finditer(r"package:\/\/([^\/]+)([^\"]+)", urdf)
    for package in packages:
        path = rospack.get_path(package.group(1))
        urdf = urdf.replace(package.group(0), f'{path}/{package.group(2)}')
        
    return urdf

class MissingParameterError(ValueError):
    pass

class IKError(RuntimeError):
    pass

class PlacoRos():
    def __init__(self):
        self.seq = 0
        self.robot_lock = Lock()
        state_lock = Lock()
        self.state_received_cond = Condition(state_lock)
        urdf = rospy.get_param('/robot_description')
        if not urdf:
            rospy.signal_shutdown()
            raise MissingParameterError("Missing /robot_description parameter, did you publish your robot's description ?")

        urdf = parse_ros_packages(urdf)
        
        visualize = rospy.get_param('~visualize', False)
        
        self.prefix = rospy.get_param('~link_prefix', '')
        self.base_frame = self.prefix + rospy.get_param('~base_frame', 'base_link')
        self.effector_frame = self.prefix + rospy.get_param('~effector_frame', 'ee_link')
        
        joint_states = rospy.get_param('~joint_state_topic', '/joint_states')
        
        self.frequency = rospy.get_param('~dt', 100.)
        self.conservative_duration = rospy.get_param('~conservative_duration', 10.0) # In seconds
        self.max_acceleration = rospy.get_param('~max_acceleration', 1.0) # In m.s^-2
        self.max_search_iter = rospy.get_param('~max_search_iter', 10) # In iterations
        self_collide = rospy.get_param('~self_collide', False)
        
        pairs = rospy.get_param('~collision_pairs', '')
        self.robot = placo.RobotWrapper('', 0, urdf)
        if pairs:
            self.robot.load_collision_pairs(pairs)
            
        self.n_joints = len(self.robot.joint_names())
        rospy.loginfo(f'Joint names : {list(self.robot.joint_names())}')
        rospy.loginfo(f'number of joints : {len(self.robot.joint_names())}')
        
        self.solver = placo.KinematicsSolver(self.robot)
        self.active_joints = [self.prefix + name for name in UNMASKED_JOINT_NAMES]
                
        self.solver.mask_fbase(True)
        self.solver.enable_velocity_limits(True)
        self.solver.enable_joint_limits(True)
        
        if self_collide:
            # Adds the constraint to the solver
            collisions_constraint = self.solver.add_avoid_self_collisions_constraint()
            # Margin to avoid self collisions
            collisions_constraint.self_collisions_margin = 0.01 # 1cm

            # Distance where the constraint starts being active in the solver
            # (to avoid extra computations for parts that are far away)
            collisions_constraint.self_collisions_trigger = 0.05 # 5cm
                
        dt = 1. / self.frequency
        self.solver.dt = dt
        
        self.tf2_buffer = tf2.Buffer(rospy.Duration(20.0))
        self.tf2_listener = tf2.TransformListener(self.tf2_buffer)
        
        self.kinematics_server = actionlib.SimpleActionServer('goal_pose', URGoToAction, self.execute_goal, auto_start=False)
        self.result = URGoToResult()
        self.feedback = URGoToFeedback()
    
        self.state_sub = rospy.Subscriber(joint_states, JointState, self.joint_state_callback)
        
        self.list_joints_srv = rospy.Service('list_active_joints', ListJoints, self.list_joints)
        self.set_active_joints_srv = rospy.Service('set_active_joints', SetActiveJoints, self.set_active_joints)
        self.mask_joints_srv = rospy.Service('mask_joints', MaskJoints, self.mask_joints)
        self.unmask_joints_srv = rospy.Service('unmask_joints', UnmaskJoints, self.unmask_joints)
            
        self.joint_state = None

        self.T_world_target = np.zeros((4,4), dtype=np.float32)
        self.effector_task = self.solver.add_frame_task(self.effector_frame, self.T_world_target)
        self.effector_task.configure("effector", "soft", 1., 1.)
        
        self.spline = placo.CubicSpline()
        
        if visualize:
            self.viz = robot_viz(self.robot, "UR5")
            self.visualization_timer = rospy.Timer(rospy.Duration(1/60), self.visualization_callback)
        
        self.kinematics_server.start()


    def execute_goal(self, goal: URGoToGoal):
        success = URGoToResult.FAILED
        trajectory = JointTrajectory()
        trajectory.joint_names = self.active_joints
        trajectory.header = Header(self.seq, goal.target_pose.header.stamp, '')
        
        rospy.logdebug(f'Received goal...')
        with self.state_received_cond:
            self.state_received_cond.wait_for(self.is_state_valid)
            self.set_robot_q(self.joint_state)
            
        dt = 1. / self.frequency
        goal_duration = goal.duration.to_sec()
        start_pose = self.robot.get_T_world_frame(self.effector_frame)
        self.T_world_target = self.pose_to_matrix(goal.target_pose)
            
        rospy.logdebug(f"Starting goal : {goal}")
        t = 0.
        start = time.monotonic()
        with self.robot_lock:
            if goal_duration > 0:
                self.init_spline(goal_duration)
                target_frames = self.precompute_trajectory_frames(start_pose, self.T_world_target, goal_duration, dt)
                try:
                    if self.solve_with_targets(trajectory, target_frames, dt, start, goal.timeout):
                        success = URGoToResult.SUCCEEDED
                except IKError:
                    return
            else:
                current_trajectory = JointTrajectory(trajectory.header, self.active_joints, [])
                next_trajectory = JointTrajectory(trajectory.header, self.active_joints, [])
                try:
                    min_duration = 0.
                    max_duration = self.conservative_duration
                    previous = False
                    reached = False
                    i = 0
                    while i < self.max_search_iter:
                        current_duration = (max_duration + min_duration) / 2.
                        rospy.logdebug(f'Current duration for search : {current_duration}')
                        self.init_spline(current_duration)
                        target_frames = self.precompute_trajectory_frames(start_pose, self.T_world_target, current_duration, dt)
                        valid_accel = self.check_accelerations(target_frames, dt)
                        
                        if valid_accel:
                            previous = reached
                            q = self.robot.state.q.copy()
                            reached = self.solve_with_targets(next_trajectory, target_frames, dt, start, goal.timeout)
                            # Reset robot model state after exploration
                            self.robot.state.q = q
                            self.robot.update_kinematics()
                            
                            if reached:
                                # Swap pointers
                                tmp = next_trajectory
                                next_trajectory = current_trajectory
                                current_trajectory = tmp
                                max_duration = current_duration
                            else:
                                min_duration = current_duration
                            next_trajectory.points.clear()
                        else:
                            min_duration = current_duration
                        i += 1
                    
                    if previous:
                        success = URGoToResult.SUCCEEDED
                        trajectory = current_trajectory
                        # print(current_trajectory)
                except IKError:
                    return
                except TimeoutError:
                    trajectory = current_trajectory
            
        self.result.state = success
        self.result.trajectory = trajectory
        self.kinematics_server.set_succeeded(self.result)
            

    def joint_state_callback(self, state: JointState) -> None:
        with self.state_received_cond:
            ## Reorder the received JointState message to match placo joint order
            self.joint_state = zip(state.name, state.position, state.velocity, state.effort)          
            self.state_received_cond.notify()


    def solve_with_targets(self, trajectory, target_frames, dt, start, timeout, update_model = True) -> bool:
        t = 0.
        for target_frame in target_frames:
            t += dt
            if (time.monotonic() - start) >= timeout:
                raise TimeoutError("Solver timeout")
            if self.kinematics_server.is_preempt_requested():
                self.handle_preempt()
                return False
            if not self.solver_tick(target_frame, update_model):
                raise IKError("Solver failed")
            
            q = self.get_robot_q(self.active_joints)
            point = JointTrajectoryPoint()
            point.positions = q
            point.time_from_start = rospy.Duration.from_sec(t)
            
            trajectory.points.append(point)
            
        return self.target_reached()


    def solver_tick(self, target_frame, update_model = True) -> bool:
        try:
            self.effector_task.T_world_frame = target_frame
            t1 = time.monotonic()
            self.solver.solve(update_model)
            t2 = time.monotonic()
            # rospy.logdebug(f'Solve iteration time : {t2 - t1}')
            self.robot.update_kinematics()
        except RuntimeError as e:
            rospy.logerr(f'IK solve failed : {e.with_traceback(None)}')
            self.result.state = self.result.FAILED
            self.kinematics_server.set_aborted(result=self.result)
            return False
        
        return True


    def check_accelerations(self, target_frames: list, dt: float) -> bool:
        v1 = (target_frames[1][:3, 3] - target_frames[0][:3, 3]) / dt
        v2 = (target_frames[2][:3, 3] - target_frames[1][:3, 3]) / dt
        
        # print(f'v1 = {v1}')
        # print(f'v2 = {v2}')
        accel = (v2 - v1) / dt
        # print(f'a = {accel}')
        
        # No need to compute deceleration since Cubic spline is symmetric at ends ?
        v1 = (target_frames[-1][:3, 3] - target_frames[-2][:3, 3]) / dt
        v2 = (target_frames[-2][:3, 3] - target_frames[-3][:3, 3]) / dt
        decel = (v2 - v1) / dt
        
        # max_accel = max(abs(np.linalg.norm(accel)), abs(np.linalg.norm(decel)))
        accel_norm = abs(np.linalg.norm(accel))
        decel_norm = abs(np.linalg.norm(decel))
        # print(f'Max acceleration : {accel_norm}')
        
        return max(accel_norm, decel_norm) < self.max_acceleration


    def get_robot_q(self, joint_names: Iterable) -> Iterable:
        robot_q = []
        for name in joint_names:
            robot_q.append(self.robot.get_joint(name))
        return robot_q
    
    
    def set_robot_q(self, joint_states: Iterable) -> None:
        for joint_name, joint_position, _, _ in joint_states:
            self.robot.set_joint(joint_name, joint_position)
        self.robot.update_kinematics()
        
        
    def init_spline(self, goal_duration: float) -> None:
        self.spline.clear()
        self.spline.add_point(0.0, 0.0, 0.0)
        self.spline.add_point(goal_duration, 1.0, 0.0)
        

    def precompute_trajectory_frames(self, start_pose, target_pose, goal_duration: float, dt: float) -> Iterable:
        t = 0.
        target_frames = []
        while t < goal_duration:
            t += dt
            frame = placo.interpolate_frames(start_pose, target_pose, self.spline.pos(t))
            target_frames.append(frame)
            
        return target_frames
    

    def visualization_callback(self, _: rospy.timer.TimerEvent) -> None:
        with self.robot_lock:
            self.viz.display(self.robot.state.q)
            robot_frame_viz(self.robot, self.effector_frame)
        if self.T_world_target is not None:
            frame_viz("target", self.T_world_target)


    def is_state_valid(self):
        return self.joint_state is not None


    def target_reached(self):
        # orient_err = self.effector_task.orientation().error_norm()
        # pos_err = self.effector_task.position().error_norm()
        current_pose = self.robot.get_T_world_frame(self.effector_frame)
        orient_err = np.linalg.norm(self.T_world_target[:, :3] - current_pose[:, :3])
        pos_err = np.linalg.norm(self.T_world_target[:, 3] - current_pose[:, 3])
        
        return orient_err < 1e-3 and pos_err < 1e-3


    def pose_to_matrix(self, pose: PoseStamped) -> np.ndarray:
        #Transform point into arm base frame
        try:
            tf = self.tf2_buffer.lookup_transform(self.base_frame, pose.header.frame_id, pose.header.stamp)
        except Exception as e:
            tf = TransformStamped()
            rospy.logerr(f'Failed to transform from frame "{pose.header.frame_id}" to frame "{self.base_frame}" : {e.with_traceback(None)}')

        pose = tf2_geometry_msgs.do_transform_pose(pose, tf)
        
        pos = pose.pose.position
        rot = pose.pose.orientation
        
        T = ptf.translation_matrix([pos.x, pos.y, pos.z])
        R = ptf.quaternion_matrix([rot.w, rot.x, rot.y, rot.z])
        
        return T @ R


    def handle_preempt(self):
        rospy.logwarn(f'URGoToAction : Preempted, aborting')
        self.result.state = URGoToResult.CANCELLED
        self.result.trajectory = JointTrajectory()
        self.kinematics_server.set_preempted(self.result)
    

    def list_joints(self, _) -> ListJointsResponse:
        return ListJointsResponse(self.robot.joint_names())
    
    
    def set_active_joints(self, request) -> SetActiveJointsResponse:
        for joint in request.joints:
            if joint not in self.robot.joint_names():
                return SetActiveJointsResponse(SetActiveJointsResponse.NOT_FOUND)
            
        self.active_joints.clear()
        with self.robot_lock:
            for joint in self.robot.joint_names():
                if joint in request.joints:
                    self.active_joints.append(joint)
                    
        return SetActiveJointsResponse(SetActiveJointsResponse.SUCCEEDED, self.active_joints)
    
    
    def mask_joints(self, request) -> MaskJointsResponse:
        pass
    
    
    def unmask_joints(self, request) -> UnmaskJointsResponse:
        pass
 

if __name__=="__main__":
    rospy.init_node(name="kinematics_server", argv=sys.argv, log_level=rospy.DEBUG)
    kinematics_server = PlacoRos()
    rospy.spin()