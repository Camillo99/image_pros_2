#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from zivid_interfaces.srv import LoadSettingsFromFile, Capture  # Adjust the imports based on your actual service types

class CallCaptureNode(Node):
    def __init__(self):
        super().__init__('call_capture')
        self.get_logger().info('Hello from Python!')

        # Initialize the service clients
        self.load_settings_service = self.create_client(LoadSettingsFromFile, '/zivid/load_settings_from_file')
        self.capture_service = self.create_client(Capture, '/zivid/capture')
 
        # Wait for the services to be available
        while not self.load_settings_service.wait_for_service(timeout_sec=1.0) or \
              not self.capture_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Services not available, waiting...')

        # Main loop to wait for user input and call the services
        self.run()

    def run(self):
        while rclpy.ok():
            user_input = input('Press Enter to call the load_settings_from_file service, or any other key to exit: ')

            # Call the load_settings_from_file service when the user presses Enter
            if user_input == '':
                load_settings_request = LoadSettingsFromFile.Request()
                load_settings_request.file_path = '/home/camillo/Documents/zivid_config03.yml'  # Set the file path as needed

                # Call the load_settings_from_file service
                load_settings_future = self.load_settings_service.call_async(load_settings_request)
                self.get_logger().info('Calling the load_settings_from_file service...')

                # Wait for the service response
                rclpy.spin_until_future_complete(self, load_settings_future)

                if load_settings_future.result() is not None:
                    self.get_logger().info('Service call succeeded')
                else:
                    self.get_logger().error('Service call failed')
            else:
                self.get_logger().info('Exiting...')
                break

        # Keep asking the user if they want to call the capture service
        while rclpy.ok():
            user_input = input('Press Enter to call the capture service, or any other key to exit: ')

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
                self.get_logger().info('Exiting...')
                break

def main(args=None):
    rclpy.init(args=args)
    node = CallCaptureNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
