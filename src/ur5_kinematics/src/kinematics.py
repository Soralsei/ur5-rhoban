import rospy

import numpy as np
from collections.abc import Iterable
import placo
from placo_utils.visualization import robot_viz, frame_viz, robot_frame_viz
from placo_utils.tf import tf as ptf

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header, Float64MultiArray
from ur5_kinematics.msg import URGoToAction, URGoToFeedback, URGoToResult, URGoToGoal

import actionlib
import tf2_ros as tf2
import tf2_geometry_msgs
import rospkg

import re, sys, time
from threading import Lock, Condition

JOINT_NAMES=[
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
    'hande_left_finger_joint',
    'hande_right_finger_joint',
]

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
        # print((package.group(0), path, package.group(2)))
        urdf = urdf.replace(package.group(0), f'{path}/{package.group(2)}')
        
    # print(urdf)
    return urdf

class MissingParameterError(ValueError):
    pass

class PlacoUR5Ros():
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
        controller_topic = rospy.get_param('~controller_topic', 'joint_group_eff_pos_controller/command')
        
        self.frequency = rospy.get_param('~dt', 1000.)
        self_collide = rospy.get_param('~self_collide', False)
        
        self.raw_state = rospy.get_param('~publish_raw_state', False)
        self.publish_intermediate = rospy.get_param('~publish_intermediate', False)
        rospy.loginfo(f'Only publish raw state : {self.raw_state}')
        rospy.loginfo(f'Publish intermediate kinematics commands : {self.publish_intermediate}')
        
        pairs = rospy.get_param('~collision_pairs', '')
        self.robot = placo.RobotWrapper('', 0, urdf)
        if pairs:
            self.robot.load_collision_pairs(pairs)
            
        self.n_joints = len(self.robot.joint_names())
        rospy.loginfo(f'Joint names : {list(self.robot.joint_names())}')
        rospy.loginfo(f'number of joints : {len(self.robot.joint_names())}')
        
        self.solver = placo.KinematicsSolver(self.robot)
        # Mask non moving joints (like gripper fingers for example)
        # unmasked = [prefix + name for name in UNMASKED_JOINT_NAMES]
        
        # for name in self.robot.joint_names():
        #     if name not in unmasked:
        #         print(f'Masking joint "{name}"')
        #         self.solver.mask_dof(name)
                
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
        
        if self.raw_state:
            self.controller_pub = rospy.Publisher(controller_topic, JointState, queue_size=10, tcp_nodelay=True)
            self.publish_callback: function = self.publish_raw_state
        else:
            self.state_sub = rospy.Subscriber(joint_states, JointState, self.joint_state_callback)
            self.controller_pub = rospy.Publisher(controller_topic, Float64MultiArray, queue_size=10, tcp_nodelay=True)
            self.publish_callback: function = self.publish_command
            
        self.joint_state = np.zeros(self.n_joints, dtype=np.float32) if self.raw_state else None

        self.T_world_target = np.zeros((4,4), dtype=np.float32)
        self.effector_task = self.solver.add_frame_task(self.effector_frame, self.T_world_target)
        self.effector_task.configure("effector", "soft", 1., 1.)
        
        if visualize:
            self.viz = robot_viz(self.robot, "UR5")
            self.visualization_timer = rospy.Timer(rospy.Duration(1/60), self.visualization_callback)
        
        self.kinematics_server.start()


    def execute_goal(self, goal: URGoToGoal):
        # freq = rospy.Rate(self.frequency)
        success = False
        
        self.T_world_target = self.pose_to_matrix(goal.target_pose)
        self.effector_task.T_world_frame = self.T_world_target
        rospy.logdebug(f'Received goal...')
        with self.state_received_cond:
            self.state_received_cond.wait_for(self.is_state_valid)
            
        start = rospy.Time.now()
        rospy.logdebug("Starting goal")
        while (rospy.Time.now()- start).to_sec() < goal.timeout:
            loop_start = rospy.Time.now()
            if self.kinematics_server.is_preempt_requested():
                # rospy.logdebug('URGoToGoalAction : Preempted')
                self.result.success = False
                self.kinematics_server.set_preempted(self.result)
                return
            with self.robot_lock():
                self.robot.update_kinematics()
                try:
                    t1 = rospy.Time.now()
                    self.solver.solve(True)
                    rospy.logdebug_throttle(0.2, f'IK solve time : {(rospy.Time.now() - t1).to_sec()}')
                except RuntimeError as e:
                    rospy.logerr(f'IK solve failed : {e.with_traceback(None)}')
                    self.result.success = False
                    self.kinematics_server.set_aborted(result=self.result)
                    return
            
            t1 = rospy.Time.now()
            reached = self.target_reached()
            rospy.logdebug_throttle(0.2, f'Reached time: {(rospy.Time.now() - t1).to_sec()}')
            
            if reached:
                rospy.logdebug_throttle(0.2, f'Total solve time : {(rospy.Time.now() - start).to_sec()}')
                success = True
                self.publish_callback()
                break
            
            if self.publish_intermediate:
                self.publish_callback()

            rospy.logdebug_throttle(0.2, f'Total loop time : {(rospy.Time.now() - loop_start).to_sec()}')
                
        self.result.success = success
        self.kinematics_server.set_succeeded(self.result)
        rospy.logdebug(f'Goal success : {success}')


    def joint_state_callback(self, state: JointState) -> None:
        with self.state_received_cond:
            ## Reorder the received JointState message to match placo joint order
            joint_state = sorted(zip(state.name, state.position), key = lambda i: list(self.robot.joint_names()).index(i[0]))
            
            for name, state in joint_state:
                self.robot.set_joint(name, state)
                
            joint_state = [list(t) for t in zip(*joint_state)][1] # Take only q and not joint names
            self.joint_state = joint_state
            
            self.state_received_cond.notify()


    def publish_command(self) -> None:
        unmasked_joints = [self.prefix + name for name in UNMASKED_JOINT_NAMES]
        robot_q = PlacoUR5Ros.get_robot_q(self.robot, unmasked_joints)
        
        q = Float64MultiArray(data=robot_q)
        
        rospy.logdebug('URGoToGoalAction : Publishing position...')
        self.controller_pub.publish(q)


    def publish_raw_state(self) -> None:
        names = self.robot.joint_names()
        Q = PlacoUR5Ros.get_robot_q(self.robot, names)
        state = JointState()
        state.header = Header(seq=self.seq, stamp=rospy.Time.now())
        state.name = names
        state.position = Q
        self.controller_pub.publish(state)


    @staticmethod
    def get_robot_q(robot: placo.RobotWrapper, joint_names: Iterable) -> Iterable:
        robot_q = []
        for name in joint_names:
            robot_q.append(robot.get_joint(name))
        return robot_q


    def visualization_callback(self, _: rospy.timer.TimerEvent) -> None:
        with self.robot_lock:
            self.viz.display(self.robot.state.q)
            robot_frame_viz(self.robot, self.effector_frame)
        if self.T_world_target is not None:
            frame_viz("target", self.T_world_target)


    def is_state_valid(self):
        return self.joint_state is not None


    def target_reached(self):
        orient_err = self.effector_task.orientation().error()
        pos_err = self.effector_task.position().error()
        return np.linalg.norm(orient_err) < 1e-5 and np.linalg.norm(pos_err) < 1e-3


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

 

if __name__=="__main__":    
    rospy.init_node(name="kinematics_server", argv=sys.argv, log_level=rospy.INFO)
    kinematics_server = PlacoUR5Ros()
    rospy.spin()