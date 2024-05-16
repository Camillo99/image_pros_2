#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from zivid_interfaces.srv import Capture  # Adjust the imports based on your actual service types

class CallCaptureNode(Node):
    def __init__(self):
        super().__init__('call_capture')
        self.get_logger().info('Hello from Python!')

        # Initialize the service clients
        self.capture_service = self.create_client(Capture, '/zivid/capture')
 
        # Wait for the services to be available
        while not self.capture_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Services not available, waiting...')

        # Main loop to wait for user input and call the services
        self.run()

    def run(self):


        # Keep asking the user if they want to call the capture service
        while rclpy.ok():
            self.get_logger().info('Press Enter to call the capture service')
            user_input = input()

            # Call the capture service when the user presses Enter
            if user_input == '':
                capture_request = Capture.Request()

                # Call the capture service
                capture_future = self.capture_service.call_async(capture_request)
                self.get_logger().info('Calling the capture service...')

                # Wait for the service response
                rclpy.spin_until_future_complete(self, capture_future)

                if capture_future.result() is not None:
                    self.get_logger().info('Capture service call succeeded')
                else:
                    self.get_logger().error('Capture service call failed')
            else:
                self.get_logger().info('nothing')

def main(args=None):
    rclpy.init(args=args)
    node = CallCaptureNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
