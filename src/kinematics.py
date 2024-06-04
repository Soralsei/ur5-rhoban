import placo
import numpy as np
import time
from placo_utils.visualization import robot_viz, frame_viz, robot_frame_viz
from placo_utils.tf import tf

robot = placo.RobotWrapper('./ur5.urdf')
T_world_target = tf.translation_matrix([0.5, 0., 0.5])

viz = robot_viz(robot)

solver = placo.KinematicsSolver(robot)

solver.mask_fbase(True)
solver.enable_velocity_limits(True)
solver.enable_joint_limits(True)

effector_task = solver.add_frame_task("ee_link", T_world_target)
effector_task.configure("effector", "soft", 1., 1.)

dt = 0.002
solver.dt = dt
t = 0
last_display = 0

while True:
    T_world_target = tf.translation_matrix([0.5, np.sin(t)*1.0, 0.5])
    effector_task.T_world_frame = T_world_target

    robot.update_kinematics()
    solver.solve(True)

    # solver.dump_status()

    # print("q:")
    # print(robot.state.q[-6:])

    if t > last_display + 0.01:
        last_display = t
        viz.display(robot.state.q)  
        robot_frame_viz(robot, "ee_link")
        frame_viz("target", T_world_target)
    time.sleep(dt)
    t += dt