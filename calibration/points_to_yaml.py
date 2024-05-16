#!/usr/bin/env python3

import math
from socket import timeout
# from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
# from builtin_interfaces.msg import Duration
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# from turtlesim.srv import Spawn
from time import sleep
import numpy as np
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

#quaternion transformation
from scipy.spatial.transform import Rotation as R
import numpy as np
import yaml

_EPS = np.finfo(float).eps * 4.0

def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.

    >>> R = quaternion_matrix([0.06146124, 0, 0, 0.99810947])
    >>> numpy.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
    True

    """
    # q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    # nq = np.dot(q, q)
    # if nq < _EPS:
    #     return np.identity(4)
    # q *= math.sqrt(2.0 / nq)
    # q = np.outer(q, q)
    # return np.array((
    #     (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], quaternion[0]),
    #     (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], quaternion[1]),
    #     (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], quaternion[2]),
    #     (                0.0,                 0.0,                 0.0, 1.0)
    #     ), dtype=np.float64)

    r = R.from_quat(quaternion[3:])
    matrix = r.as_matrix()
    trans = np.array([quaternion[0], quaternion[1], quaternion[2]])
    matrix = np.column_stack((matrix, trans))
    matrix = np.row_stack((matrix, [0,0,0,1]))
    return matrix


if __name__ == '__main__':
    pose = [-0.013081211536721288, 0.4028389138025321, 0.35582518337635216, 0.03644065129577184, 0.9993271905525889, -0.0034583585765795323, -0.0022988935089452065]
    matrix = quaternion_matrix(pose)
    print(str(matrix))
    data = {
            "__version__": {
                "serializer": 1,
                "data": 1
            },
            "FloatMatrix": {
                "Data": str([
        [matrix[0,0], matrix[0,1], matrix[0,2], matrix[0,3]],
        [matrix[1,0], matrix[1,1], matrix[1,2], matrix[1,3]],
        [matrix[2,0], matrix[2,1], matrix[2,2], matrix[2,3]],
        [matrix[3,0], matrix[3,1], matrix[3,2], matrix[3,3]]])
            }
        }
    
    # Function to represent float values in the YAML file in scientific notation
    def float_representer(dumper, value):
        text = '{:0.16e}'.format(value)
        return dumper.represent_scalar(u'tag:yaml.org,2002:float', text)

    # Add the float representer to the YAML dumper
    yaml.add_representer(float, float_representer)

    def represent_data(dumper, data):
        return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)

    yaml.add_representer(list, represent_data)


    with open("/home/camillo/Documents/calibration/pos01.yaml", "w") as yaml_file:
        yaml.dump(data, yaml_file, default_flow_style=False, sort_keys=False)



    


    