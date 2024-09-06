from functools import reduce
import rospy

import numpy as np
from collections.abc import Iterable
import placo, pinocchio

import rospkg

import re, sys, json

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

class ConfigurationSampler():
    def __init__(self, prefix: str):
        urdf = rospy.get_param('/robot_description')
        if not urdf:
            rospy.signal_shutdown()
            raise MissingParameterError("Missing /robot_description parameter, did you publish your robot's description ?")
        
        # print(f"URDF :\n{urdf}")
        urdf = parse_ros_packages(urdf)
        
        self.robot = placo.RobotWrapper('', 0, urdf)
        self.n_joints = len(self.robot.joint_names())
        self.prefix: str = prefix
        
        self.joint_names = [self.prefix + name for name in UNMASKED_JOINT_NAMES]
        
        rospy.loginfo(f'Joint names : {list(self.robot.joint_names())}')
        rospy.loginfo(f'number of joints : {len(self.robot.joint_names())}')
    
    def random_configuration(self) -> None:
        for name in self.robot.joint_names():
            if name not in self.joint_names:
                continue
            r = np.random.rand()
            # Number in [-1, 1] interval
            r = r -(1 - r)
            self.robot.set_joint(name, r * np.pi)
        self.robot.update_kinematics()
    
    def get_collision_pairs(self):
        return self.robot.collision_model.collisionPairs.tolist()
    
    def sample_collision_pairs(self) -> Iterable:
        pairs = []
        parents = self.robot.model.parents.tolist()[1:]
        subtrees = [subtree.tolist() for subtree in self.robot.model.subtrees.tolist()]
        children = []
        ## Recreate model tree from subtrees
        
        # Get unique subtrees in robot model
        for subtree in subtrees:
            if subtree not in children:
                children.append(subtree)

        # Remove redundant children in subtrees
        for i in range(len(children) - 1):
            a = set(children[i])
            for j in range(i + 1, len(children)):
                a = a - set(children[j])
            children[i] = list(a)
        
        # Regroup children with same parent in same list
        for i in range(0, len(parents) - 1):
            if parents[i] == parents[i + 1]:
                children[parents[i]].extend(children.pop(parents[i] + 1))
        
        parents = list(set(parents))
        consecutive = []
        buffer = []
        buffer.append(parents[0])
        visited = [False] * reduce(lambda count, l: count + len(l), children, 0)
        
        # Depth first link tree traversal
        while buffer:
            node = buffer.pop()
            if node < len(children) and not visited[node]:
                visited[node] = True
                for child in children[node]:
                    consecutive.append((node, child))
                    buffer.append(child)
        print(f'Consecutive pairs in model : {consecutive}')
    
        # pairs = [(pair.first, pair.second) for pair in self.robot.collision_model.collisionPairs if (pair.first, pair.second) not in consecutive]
        # for pair in consecutive:
        #     print(pair)
        #     p = pinocchio.CollisionPair(pair[0], pair[1])
        #     self.robot.collision_model.removeCollisionPair(p)
        
        pairs = []
        for _ in range(10_000):
            self.random_configuration()
            collisions: list[placo.Collision] = list(self.robot.self_collisions(False))
            collisions = [(collision.objA, collision.objB) for collision in collisions]
            pairs.extend(collisions)
        
        actual = [collision for collision in pairs if collision not in consecutive]
        
        return set(actual)


if __name__=="__main__":
    import argparse, os
    
    parser = argparse.ArgumentParser()
    parser.add_argument('-o', '--output' , type=str, required=True)
    parser.add_argument('-p', '--prefix' , type=str, default='')
    args = parser.parse_args()
    
    rospy.init_node(name="kinematics_server", argv=sys.argv, log_level=rospy.INFO)
    sampler = ConfigurationSampler(args.prefix)
    print(f'Default collision pairs count : {len(sampler.get_collision_pairs())}')
    pairs = sampler.sample_collision_pairs()
    print(f'Sampled collision pairs count : {len(pairs)}')
    
    with open(args.output, 'w', encoding='utf-8') as f:
        json.dump(sorted(list(pairs)), f)
        path = args.output if os.path.isabs(args.output) else f'{os.getcwd()}/{args.output}'
        print(f'Wrote collision pairs to {path}')