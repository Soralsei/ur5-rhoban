import placo
import rospy
import numpy as np
import time
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

import re
from threading import Lock, Condition

JOINT_NAMES=[
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
    "hande_left_finger_joint",
    "hande_right_finger_joint",
]


def parse_ros_packages(urdf: str):
    if not urdf:
        raise TypeError(f"parse_ros_packages: URDF missing")
    
    rospack = rospkg.RosPack()
    
    packages = re.finditer(r"package:\/\/([^\/]+)([^\"]+)", urdf)
    
    for package in packages:
        path = rospack.get_path(package.group(1))
        print((package.group(0), path, package.group(2)))
        urdf = urdf.replace(package.group(0), f'{path}/{package.group(2)}')
        
    # print(urdf)
    return urdf

class MissingArgumentError(ValueError):
    pass

class PlacoUR5Ros():
    def __init__(self):
        self.robot_lock = Lock()
        state_lock = Lock()
        self.state_received_cond = Condition(state_lock)
        urdf = rospy.get_param('/robot_description')
        if not urdf:
            rospy.signal_shutdown()
            raise MissingArgumentError("Missing /robot_description parameter, did you publish your robot's description ?")
        
        # print(f"URDF :\n{urdf}")
        urdf = parse_ros_packages(urdf)
        
        self.robot = placo.RobotWrapper('', 0, urdf)
        self.n_joints = len(self.robot.joint_names())
        self.viz = robot_viz(self.robot, "UR5")
        
    
        print(f'Joint names : {list(self.robot.joint_names())}')
        print(f'number of joints : {len(self.robot.joint_names())}')
        self.solver = placo.KinematicsSolver(self.robot)
        
        self.solver.mask_fbase(True)
        self.solver.enable_velocity_limits(True)
        self.solver.enable_joint_limits(True)
        
        self.tf2_buffer = tf2.Buffer(rospy.Duration(20.0))
        self.tf2_listener = tf2.TransformListener(self.tf2_buffer)
        
        self.kinematics_server = actionlib.SimpleActionServer('goal_pose', URGoToAction, self.execute_goal, auto_start=False)
        self.result = URGoToResult()
        self.feedback = URGoToFeedback()
        
        self.state_sub = rospy.Subscriber('joint_states', JointState, self.joint_state_callback)
        self.controller_pub = rospy.Publisher('/joint_group_eff_pos_controller/command', Float64MultiArray, queue_size=10)
        self.joint_state = None

        self.T_world_target = ptf.translation_matrix([0.492, 0.134, 0.484])
        self.effector_task = self.solver.add_frame_task("hande_right_finger", self.T_world_target)
        self.effector_task.configure("effector", "soft", 1., 1.)
        
        self.visualization_timer = rospy.Timer(rospy.Duration(1/30), self.visualization_callback)
        
        self.kinematics_server.start()
    
    def execute_goal(self, goal: URGoToGoal):
        freq = rospy.Rate(1000)
        dt = 1. / 1000.
        success = False
        self.solver.dt = dt
        
        self.T_world_target = self.pose_to_matrix(goal.target_pose)
        self.effector_task.T_world_frame = self.T_world_target
        rospy.loginfo(f'Received goal {goal}')
        with self.state_received_cond:
            self.state_received_cond.wait_for(self.is_state_valid)
            
            start = rospy.Time.now()
            while (rospy.Time.now()- start).to_sec() < goal.timeout:
                
                if self.kinematics_server.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self.kinematics_server.set_preempted()
                    success = False
                    return
                t1 = time.time()
                self.robot.update_kinematics()
                try:
                    self.solver.solve(True)
                except RuntimeError as e:
                    print(f'IK solve failed : {e.with_traceback(None)}')
                t2 = time.time()
                rospy.logdebug_once(f'Placo solve time : {t2 - t1}')
                
                robot_q = []
                for name in self.robot.joint_names():
                    robot_q.append(self.robot.get_joint(name))
                print(robot_q[-2:])
                q = Float64MultiArray(data=robot_q[:-2])
                self.controller_pub.publish(q)
                
                if self.effector_task.orientation().error_norm() <= 1e-2 and self.effector_task.position().error_norm() <= 1e-3:
                    success = True
                    break
                
                # print("Waiting for controller to reach the position...")
                # self.state_received_cond.wait_for(np.isclose(self.joint_state[-1:], robot_q, atol=1e-3))
                
                freq.sleep()
        self.result.success = success
        rospy.loginfo(f'Goal success : {success}')
        self.kinematics_server.set_succeeded(self.result)    
    
    def joint_state_callback(self, state: JointState) -> None:
        with self.state_received_cond:
            ## Reorder the received JointState message to match placo joint order
            joint_state = sorted(zip(state.name, state.position), key = lambda i: list(self.robot.joint_names()).index(i[0]))
            if self.joint_state == None:
                for name, state in joint_state:
                    self.robot.set_joint(name, state)
            joint_state = [list(t) for t in zip(*joint_state)][1] # Take only q and not joint names
            self.joint_state = joint_state
            self.state_received_cond.notify()
    
    def visualization_callback(self, event: rospy.timer.TimerEvent) -> None:
        with self.robot_lock:
            self.viz.display(self.robot.state.q)
            robot_frame_viz(self.robot, "hande_right_finger")
            frame_viz("target", self.T_world_target)

    def is_state_valid(self):
        return self.joint_state is not None

    def controller_reached(self):
        return np.isclose(self.joint_state[-1:], atol=1e-3)

    def pose_to_matrix(self, pose: PoseStamped) -> np.ndarray:
        #Transform point into arm base frame
        try:
            tf = self.tf2_buffer.lookup_transform("base_link", pose.header.frame_id, pose.header.stamp)
        except Exception as e:
            tf = TransformStamped()
            rospy.logerr(f'Failed to transform from frame "{pose.header.frame_id}" to frame "base_link" : {e.with_traceback(None)}')

        pose = tf2_geometry_msgs.do_transform_pose(pose, tf)
        
        pos = pose.pose.position
        rot = pose.pose.orientation
        
        T = ptf.translation_matrix([pos.x, pos.y, pos.z])
        R = ptf.quaternion_matrix([rot.w, rot.x, rot.y, rot.z])
        
        return T @ R
    

if __name__=="__main__":
    import signal
    
    running = True

    # def sig_handler(sig_num, _):
    #     global running
    #     running = False
    #     rospy.signal_shutdown("Sigint")
    
    # signal.signal(signal.SIGINT, sig_handler)
    
    rospy.init_node("ur5e_kinematics")
    test = PlacoUR5Ros()
    # urdf = rospy.get_param('/robot_description')
    # if not urdf:
    #     rospy.signal_shutdown()
    #     raise MissingArgumentError("Missing /robot_description parameter, did you publish your robot's description ?")
    
    # # print(f"URDF :\n{urdf}")
    # urdf = parse_ros_packages(urdf)
    
    # robot = placo.RobotWrapper('', 0, urdf)
    # viz = robot_viz(robot, "UR5")
    # T_world_target = ptf.translation_matrix([0.492, 0.134, 0.484])

    # solver = placo.KinematicsSolver(robot)
    
    # print(f'Joint names : {list(robot.joint_names())}')
    # print(f'number of joints : {len(robot.joint_names())}')

    # solver.mask_fbase(True)
    # solver.enable_velocity_limits(True)
    # solver.enable_joint_limits(True)

    # effector_task = solver.add_frame_task("hande_left_finger_joint", T_world_target)
    # effector_task.configure("effector", "hard", 1., 1.)

    # dt = 0.002
    # solver.dt = dt
    # t = 0
    # last_display = 0

    # pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    
    # seq = 0
    
    # n_joints = len(robot.joint_names())
    
    # header = Header()
    # header.stamp = rospy.get_rostime()
    # header.seq = seq
    # state = JointState()
    # state.header = header
    # state.name = robot.joint_names()
    # state.position = robot.state.q[-n_joints:]
    
    # pub.publish(state)
    
    # previous_state = robot.state.q[-n_joints:]
    # stagnant = 0

    # print(f"Starting...")
    # exit(0)
    # while not rospy.is_shutdown() and running:
    #     # T_world_target = ptf.translation_matrix([0.35, 1.0 * np.sin(t), 0.25]) @ ptf.euler_matrix(np.pi, 0, 0)
    #     T_world_target = ptf.translation_matrix([0.35, 2, 0.5]) @ ptf.euler_matrix(np.pi, 0, 0)
    #     effector_task.T_world_frame = T_world_target

    #     # q = [0., -1.57, 1.57, -1.57, -1.57, -1.57, 0., 0.]
    #     # for name, angle in zip(JOINT_NAMES, q):
    #     #     robot.set_joint(name, angle)
    #     robot.update_kinematics()
    #     try:
    #         solver.solve(True)
    #     except RuntimeError as e:
    #         print(f'IK solve failed : {e.with_traceback(None)}')

    #     print(f'previous : {previous_state}\ncurrent: {robot.state.q[-n_joints:]}')
    #     if np.allclose((robot.state.q[-n_joints:]), previous_state, atol=1e-3):
    #         print(f'Stagnant {stagnant}')
    #         stagnant += 1
    #     else:
    #         stagnant = 0
        
    #     if stagnant >= 10 and abs(np.mean(effector_task.position().error())) >= 0.1:
    #         print("Out of reach ?")
    #         stagnant = 0
    #     # solver.dump_status()

    #     # print("q:")
    #     # print(f"q: {robot.state.q[-6:]}")
        
    #     header = Header()
    #     header.stamp = rospy.get_rostime()
    #     header.seq = seq
    #     state = JointState()
    #     state.header = header
    #     state.name = robot.joint_names()
    #     state.position = robot.state.q[-n_joints:]
        
    #     pub.publish(state)
        
    #     previous_state = robot.state.q[-n_joints:]
        
    #     if t > last_display + 0.01:
    #         last_display = t
    #         viz.display(robot.state.q)  
    #         robot_frame_viz(robot, "hande_left_finger_joint")
    #         frame_viz("target", T_world_target)
        
    #     # print(robot.get_T_world_frame("hande_left_finger_joint"))
    #     time.sleep(dt)
    #     t += dt
    #     seq += 1