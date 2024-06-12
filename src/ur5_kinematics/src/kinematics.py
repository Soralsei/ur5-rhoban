import placo
import rospy
import numpy as np
import time
from placo_utils.visualization import robot_viz, frame_viz, robot_frame_viz
from placo_utils.tf import tf

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

import actionlib

import rospkg

import re
from threading import Lock

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
        urdf = rospy.get_param('/robot_description')
        if not urdf:
            rospy.signal_shutdown()
            raise MissingArgumentError("Missing /robot_description parameter, did you publish your robot's description ?")
        
        # print(f"URDF :\n{urdf}")
        urdf = parse_ros_packages(urdf)
        
        self.robot = placo.RobotWrapper('', 0, urdf)
        self.viz = robot_viz(robot, "UR5")
        
        self.solver = placo.KinematicsSolver(robot)
        
        self.solver.mask_fbase(True)
        self.solver.enable_velocity_limits(True)
        self.solver.enable_joint_limits(True)
        
        self.kinematics_server = actionlib.SimpleActionServer('goal_pose', PoseStamped, self.execute_goal)

        self.effector_task = self.solver.add_frame_task("hande_left_finger_joint", T_world_target)
        self.effector_task.configure("effector", "soft", 1., 1.)
        
        self.visualization_timer = rospy.Timer(rospy.Duration(1/30), self.visualization_callback)
        
        self.kinematics_server.start()
    
    def execute_goal(self, goal: PoseStamped):
        pass
    
    def visualization_callback(self, event: rospy.timer.TimerEvent) -> None:
        with self.robot_lock:
            self.viz.display(self.robot.state.q)
            robot_frame_viz(self.robot, "hande_left_finger_joint")
            frame_viz("target", self.T_world_target)
    
    

if __name__=="__main__":
    import signal
    
    running = True

    def sig_handler(sig_num, _):
        global running
        running = False
    
    signal.signal(signal.SIGINT, sig_handler)
    
    rospy.init_node("ur5e_kinematics")
    urdf = rospy.get_param('/robot_description')
    if not urdf:
        rospy.signal_shutdown()
        raise MissingArgumentError("Missing /robot_description parameter, did you publish your robot's description ?")
    
    # print(f"URDF :\n{urdf}")
    urdf = parse_ros_packages(urdf)
    
    robot = placo.RobotWrapper('', 0, urdf)
    viz = robot_viz(robot, "UR5")
    T_world_target = tf.translation_matrix([0.492, 0.134, 0.484])

    solver = placo.KinematicsSolver(robot)
    
    print(f'Joint names : {list(robot.joint_names())}')
    print(f'number of joints : {len(robot.joint_names())}')

    solver.mask_fbase(True)
    solver.enable_velocity_limits(True)
    solver.enable_joint_limits(True)

    effector_task = solver.add_frame_task("hande_left_finger_joint", T_world_target)
    effector_task.configure("effector", "soft", 1., 1.)

    dt = 0.01
    solver.dt = dt
    t = 0
    last_display = 0

    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    seq = 0
    
    n_joints = len(robot.joint_names())

    print(f"Starting...")
    while not rospy.is_shutdown() and running:
        T_world_target = tf.translation_matrix([0.75, 1.0 * np.sin(t), 0.25]) @ tf.euler_matrix(np.pi, 0, 0)
        effector_task.T_world_frame = T_world_target

        # q = [0., -1.57, 1.57, -1.57, -1.57, -1.57, 0., 0.]
        # for name, angle in zip(JOINT_NAMES, q):
        #     robot.set_joint(name, angle)
        robot.update_kinematics()
        try:
            solver.solve(True)
        except RuntimeError as e:
            print(f'IK solve failed : {e.with_traceback(None)}')

        # solver.dump_status()

        # print("q:")
        # print(f"q: {robot.state.q[-6:]}")
            
        header = Header()
        header.stamp = rospy.get_rostime()
        header.seq = seq
        state = JointState()
        state.header = header
        state.name = robot.joint_names()
        state.position = robot.state.q[-n_joints:]
        
        pub.publish(state)
        
        if t > last_display + 0.01:
            last_display = t
            viz.display(robot.state.q)  
            robot_frame_viz(robot, "hande_left_finger_joint")
            frame_viz("target", T_world_target)
        
        # print(robot.get_T_world_frame("hande_left_finger_joint"))
        time.sleep(dt)
        t += dt
        seq += 1