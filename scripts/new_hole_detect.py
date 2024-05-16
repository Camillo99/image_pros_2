#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseArray, Pose
import pickle
from scipy.spatial.transform import Rotation as Rot
import numpy as np



class HoleDetector(Node):
    def __init__(self):
        super().__init__('new_hole_detect')

        self.publisher = self.create_publisher(PoseArray, 'hole_wrt_base', 10)
        self.publisher_2 = self.create_publisher(PoseArray, 'obj_frame_wrt_base', 10)
        self.publisher_3 = self.create_publisher(PoseArray, 'screw_hole_wrt_base', 10)
        self.publisher_4 = self.create_publisher(PoseArray, 'hole_wrt_base_link', 10)

        self.subscription1 = self.create_subscription(
            Float64MultiArray,
            '/hole_color',
            self.callback1,
            10)
        self.subscription2 = self.create_subscription(
            PoseArray,
            '/hole_pose',
            self.callback2,
            10)
        self.subscription3 = self.create_subscription(
            Float64MultiArray,
            '/Current_EE_position',
            self.callback3,
            10)
        
        # Class variables to store the received messages
        self.hole_color = None
        self.hole_pose = None
        self.EE_pose = None
        
        # Flag to indicate if both subscriptions are active
        self.hole_color_ready = False
        self.hole_pose_ready = False
        self.EE_pose_flag = False

        # Read data from Memory_2.pkl file
        memory_path = '/home/camillo/workspace/Learning/scripts/Memory_2.pkl'
        with open(memory_path, 'rb') as f:
            self.memory = pickle.load(f)
        self.get_logger().info("Data from pickle saved")
        # print(self.memory)
        self.get_logger().info("Node ready, waiting for topics publication...")
        
    def callback1(self, msg):
        self.hole_color = msg.data
        self.get_logger().info("Hole color received")
        # print(self.hole_color)
        self.hole_color_ready = True
        if self.hole_pose_ready and self.hole_color_ready:
            self.process_data()

    def callback2(self, msg):
        self.hole_pose = msg
        self.get_logger().info("Hole pose received")
        # print(self.hole_pose)
        self.hole_pose_ready = True
        if self.hole_pose_ready and self.hole_color_ready:
            self.process_data()
    
    def callback3(self, msg):
        self.EE_pose = msg.data
        # self.get_logger().info("EE_pose received")
        # print(self.EE_pose)
            
    def process_data(self):
        """Once both subscriptions are ready and callbacks are called, proceed with further operations"""
        if self.hole_color is not None and self.hole_pose is not None:
            self.get_logger().info("Ready to proceed !")
            
            # print(self.EE_pose)

            """reproject the new object marker holes on the base frame"""
            hole_wrt_base = self.hole_in_base_frame()
            
            """publish the pose of hole wrt the base frame"""
            self.pub_hole_wrt_base(hole_wrt_base)
             
            """ define the object frame w.r.t. base frame
                the frame is defined as a transformation matrix w.r.t. the base frame of the robot"""
            bTobj = self.object_frame(hole_wrt_base)

            """ reproject the holes to be screwed saved in memory to the new base frame"""
            HOLEwrtB = self.holes_in_base_frame(bTobj)

            """ impose a particular orientation to the frame of each hole in order to define
                a set of waypoints for the trajectory planner """
            self.holes_frame_for_traj_planner(HOLEwrtB)

            # set to false the flags
            self.hole_color_ready = False
            self.hole_pose_ready = False
            self.get_logger().info("\n___________________________________\n_____Data processing completed_____\n___________________________________")


    def hole_in_base_frame(self):
        """
        compute the pose of each hole wrt base frame
        holes: get from memory
        transformation between camera and EE ins constant
        transformation between EE and base is obtained from /Current_EE_position topic
        return: array of poses (xyz + quaternion)
        """
        hole_poses = []
        # print(self.EE_pose)
        for i in range(len(self.hole_pose.poses)):
            x = self.hole_pose.poses[i].position.x
            y = self.hole_pose.poses[i].position.y
            z = self.hole_pose.poses[i].position.z
            # qx = 0
            # qy = 0
            # qz = 0
            # qw = 1
            # c = memory_2['hole_color'][i]
            """
            here the orientation of the EE during the image acquisition phase 
            is used instead of the orientation obtained
            using the normal surface of the each hole
            why? --> at this phase the normal comutation is not reliable and the obtained orientation is not usable
            """
            qx = self.EE_pose[3]
            qy = self.EE_pose[4]
            qz = self.EE_pose[5]
            qw = self.EE_pose[6]
            #the color marker is added to the hole_poses data structure as the 8th element
            c = self.hole_color[i]
            hole_poses.append([x,y,z,qx,qy,qz,qw,c])
        # print(hole_poses)

        #camera frame w.r.t. EE frame --> constant transformation
        CwrtEE =  [0.066, 0.097, -0.108, -0.074, 0.000, 0.997, -0.000] # got it from the ros2 run tf2_ros tf2_echo tool0 zivid_camera

        # for each holes reprorject the pose (quaternion) from the camera frame to the base frame
        hole_wrt_base = []
        for i in range(len(hole_poses)):
            new_pose = self.transformation_to_base(hole_poses[i], CwrtEE)
            hole_wrt_base.append(new_pose)
        hole_wrt_base = np.array(hole_wrt_base) #cast to np.Array

        # print('hole_wrt_base', hole_wrt_base)
        return hole_wrt_base

    def pub_hole_wrt_base(self, hole):
        """
        publish the hole as PoseArray (useful for rviz)
        hole: xyz + quaternion + color identifier
        publish on topic: hole_wrt_base
        """
        hole = hole[:, :-1]
        # print('hole\n',hole)

        pose_array_msg = PoseArray()
        pose_array_msg.header.frame_id = 'base'  # Set the frame ID

        for pose_values in hole:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = pose_values[:3]  # Extract position
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = pose_values[3:]  # Extract quaternion
            pose_array_msg.poses.append(pose)
        
        self.publisher.publish(pose_array_msg)

    def object_frame(self, hole_wrt_base):
        """
        Get all the holes w.r.t. the base frame
        Filter the red and blue marker to define an origin frame for the box
        origin is on the red marked hole
        z-axis pointing upward --> parallel to the base frame z-axis
        x-axis pass trougth the blue marked hole
        y-axis according to right-hand-rule
        return: transformation matrix object frame w.r.t. base frame

        """
        # filter the pose of the red and blue marker holes
        red_pose = hole_wrt_base[np.argwhere(hole_wrt_base[:,-1]==1.0)] # remove the last element (color marker no longer needed)
        blue_pose = hole_wrt_base[np.argwhere(hole_wrt_base[:,-1]==3.0)]

        # attach a frame to the ojbect with:
        #   origin in the red marked hole
        #   z-axis pointing upward --> paralel to the base frame z-axis
        #   x-axis pass trougth the blue marked hole

        direction = np.array([blue_pose[0][0][0] - red_pose[0][0][0], blue_pose[0][0][1] - red_pose[0][0][1]])
        # print(direction)

        normalize_direction = self.normalize_vector(direction)

        # print(normalize_direction)

        bTobj = np.zeros((4,4))
        bTobj[0,0]= normalize_direction[0]
        bTobj[0,1]= -normalize_direction[1]
        bTobj[1,0]= normalize_direction[1]
        bTobj[1,1]= normalize_direction[0]
        bTobj[2,2] = 1
        bTobj[3,3] = 1
        bTobj[:3,3] = red_pose[0][0][:3]

        # print('bTobj\n',bTobj)

        #convert to pose (postition+quaternion)
        rot_mat1 = Rot.from_matrix(bTobj[:3,:3])  #extract the rotatinal part from the T matrix
        quat1 = rot_mat1.as_quat()  # convert from rot matrix to quaternion
        position1 = bTobj[:3,3] # extract the translational part from the T matrix
        # pose of the hole in quaternion w.r.t. the base frame + the color marker as last element of the vector
        OBJwrtB= np.concatenate((position1, quat1))
        
        # print('OBJwrtB\n', OBJwrtB)

        pose_array_msg = PoseArray()
        pose_array_msg.header.frame_id = 'base'  # Set the frame ID

        pose = Pose()
        pose.position.x = OBJwrtB[0]
        pose.position.y = OBJwrtB[1]
        pose.position.z = OBJwrtB[2]
        pose.orientation.x = OBJwrtB[3]
        pose.orientation.y = OBJwrtB[4]
        pose.orientation.z = OBJwrtB[5]
        pose.orientation.w = OBJwrtB[6]

        pose_array_msg.poses.append(pose)

        self.publisher_2.publish(pose_array_msg)

        return bTobj

    def holes_in_base_frame(self,bTobj):
        """
        bTobj: transformation matrix of object frame w.r.t. base frame
        from memory gets the pose of the green hole in object frame (xyz + quaternion)
        return: poses (xyz + quaternion) of the green holes in base frame using the data stored in memory

        """
        
        # from memory get the stored pose of the holes in object frame (as a quaternion)
        HOLEwrtOBJ = self.memory['sorted_holes']
        # print('HOLEwrtOBJ\n',HOLEwrtOBJ)
        # reproject to base frame
        HOLEwrtB= []
        for i in range(len(HOLEwrtOBJ)):
            new_pose,_ = self.pose_reprojection(HOLEwrtOBJ[i], bTobj)
            HOLEwrtB.append(new_pose)
        # print('HOLEwrtB\n',HOLEwrtB)
        HOLEwrtB = np.array(HOLEwrtB) #cast to np.Array

        #convert to PoseArray and pubblish
        pose_array_msg = PoseArray()
        pose_array_msg.header.frame_id = 'base'  # Set the frame ID

        for pose_values in HOLEwrtB:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = pose_values[:3]  # Extract position
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = pose_values[3:]  # Extract quaternion
            pose_array_msg.poses.append(pose)

        self.publisher_3.publish(pose_array_msg)
        return HOLEwrtB

    def holes_frame_for_traj_planner(self, HOLEwrtB):
        """
        HOLEwrtB: holes poses (xyz+quat) w.r.t. base frame
        the idea is to have the EE pointing downward with the camera on the outer side w.r.t. the robot
        the holes also have to be reprojected from base frame to base_link frame (required by moveit)
        publish the poses of the green holes on topic: hole_wrt_base_link
        """
        # first step reproject the holes frame from base frame to base_link
        # matrix from tf2 node
        blTb = np.array([
                            [-1.000, -0.000, 0.000, 0.000],
                            [0.000, -1.000, 0.000, 0.000],
                            [0.000, 0.000, 1.000, 0.000],
                            [0.000, 0.000, 0.000, 1.000]
                        ])
        HOLEwrtBL= []
        for i in range(len(HOLEwrtB)):
            new_pose,_ = self.pose_reprojection(HOLEwrtB[i], blTb)
            HOLEwrtBL.append(new_pose)
        HOLEwrtBL = np.array(HOLEwrtBL) #cast to np.Array

        # second step forcing the orientation of the frame in order to have the desired EE orientation
        # impose the orentation as 180 degree about x-axis --> change of the quat values
        HOLE_EE_ORIwrtBL = []
        for i in range(len(HOLEwrtBL)):
            new_hole = np.array([HOLEwrtBL[i][0],HOLEwrtBL[i][1], HOLEwrtBL[i][2], 1, 0, 0, 0 ])
            HOLE_EE_ORIwrtBL.append(new_hole)
        HOLE_EE_ORIwrtBL = np.array(HOLE_EE_ORIwrtBL)

        print('Pose of the green holes for the trajectory planner node\nHOLE_EE_ORIwrtBL\n',HOLE_EE_ORIwrtBL)

        

        #convert to PoseArray and pubblish
        pose_array_msg = PoseArray()
        pose_array_msg.header.frame_id = 'base_link'  # Set the frame ID

        for pose_values in HOLE_EE_ORIwrtBL:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = pose_values[:3]  # Extract position
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = pose_values[3:]  # Extract quaternion
            pose_array_msg.poses.append(pose)

        self.publisher_4.publish(pose_array_msg)

    # some functions for Tmatrix and quaternion handling
    def pose_reprojection(self, pose, transformation ):
        # pose --> a frame pose defined as xyz and a quaternion xyzw
        # transformation --> a transformation matrix
        # reproject the given pose between the transformation given as a matrix

        # convert the pose in a trasformation matrix
        poseT = self.Pose_2_mat(pose)

        new_poseT = np.matmul(transformation, poseT)

        # convert the new transformation matrix in a pose xyz + quaternion
        rot_mat1 = Rot.from_matrix(new_poseT[:3,:3])  #extract the rotatinal part from the T matrix
        quat1 = rot_mat1.as_quat()  # convert from rot matrix to quaternion
        position1 = new_poseT[:3,3] # extract the translational part from the T matrix
        new_pose= np.concatenate((position1, quat1))

        return new_pose, new_poseT

    def normalize_vector(self, v):
        """Normalize a vector"""
        norm = np.linalg.norm(v)
        if norm == 0:
            return v
        return v / norm

    def quaternion_matrix(self, quaternion):
        r = Rot.from_quat(quaternion)
        matrix = r.as_matrix()
        matrix = np.column_stack((matrix, [0,0,0]))
        matrix = np.row_stack((matrix, [0,0,0,1]))
        return matrix

    def Pose_2_mat(self, p):
        T = self.quaternion_matrix(p[3:])
        T[:3,3] = p[:3]
        return T          

    def transformation_to_base(self, hole_wrt_camera, CwrtEE):
        # 'T' stand for transformation matrix (4x4)

        # hole w.r.t. camera frame
        cTo = self.Pose_2_mat(hole_wrt_camera[:-1])

        # camera w.r.t EE frame
        eeTc = self.Pose_2_mat(CwrtEE)

        # EE w.r.t. base frame
        bTee = self.Pose_2_mat(self.EE_pose)

        # hole w.r.t. EE frame
        eeTo = np.matmul(eeTc, cTo)

        # object w.r.t. base frame
        bTo = np.matmul(bTee, eeTo)

        rot_mat1 = Rot.from_matrix(bTo[:3,:3])  #extract the rotatinal part from the T matrix
        quat1 = rot_mat1.as_quat()  # convert from rot matrix to quaternion
        position1 = bTo[:3,3] # extract the translational part from the T matrix
        # pose of the hole in quaternion w.r.t. the base frame + the color marker as last element of the vector
        new_pose= np.concatenate((position1, quat1, [hole_wrt_camera[-1]]))

        return new_pose



def main(args=None):
    rclpy.init(args=args)
    node = HoleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
