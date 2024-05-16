#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from zivid_interfaces.srv import LoadSettingsFromFile, Capture  # Adjust the imports based on your actual service types
from std_msgs.msg import Int16

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
        
        #create a listener for the flag from ros1 topic to trigger the capture service
        self.subscription = self.create_subscription(
            Int16,
            'Demo_status',
            self.listener_callback,
            10)

    #listener callback definition
    #when the message '1' is pubblished on the topic --> call the capture service
    #otherwise do nothing
    #due to problem with spin_until_future_complete inside of a callback,
    #the callback do not wait for a responce from the zivid service host
    def listener_callback(self, msg):
        status = msg.data
        if status == 1:
            capture_request = Capture.Request()

            # Call the capture service
            capture_future = self.capture_service.call_async(capture_request)
            self.get_logger().info('Calling the capture service...')


def main(args=None):
    rclpy.init(args=args)
    node = CallCaptureNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
