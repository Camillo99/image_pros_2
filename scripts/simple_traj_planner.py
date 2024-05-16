#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray

class SimpleTrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('simple_traj_planner')
        self.subscription = self.create_subscription(
            PoseArray,
            '/hole_pose',
            self.hole_pose_callback,
            10)
        self.subscription  # prevent unused variable warning

    def hole_pose_callback(self, msg):
        self.get_logger().info('Received %d hole poses' % len(msg.poses))
        
        
        
        for index, pose in enumerate(msg.poses):
            self.get_logger().info('Pose %d: x=%f, y=%f, z=%f' % (
                index+1, pose.position.x, pose.position.y, pose.position.z))

        #convert the received poses in something for moveit...??
        






def main(args=None):
    rclpy.init(args=args)

    simple_traj_planner = SimpleTrajectoryPlanner()

    rclpy.spin(simple_traj_planner)

    simple_traj_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
