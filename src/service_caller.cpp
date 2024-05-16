#include "rclcpp/rclcpp.hpp"
#include "image_pros/srv/hole_center.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);


    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_caller");
    rclcpp::Client<image_pros::srv::HoleCenter>::SharedPtr client =
        node->create_client<image_pros::srv::HoleCenter>("hole_center");

    auto request = std::make_shared<image_pros::srv::HoleCenter::Request>();


    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "recevied");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }

    std::vector<short int> center_x = result.get()-> hole_centers_x;
    std::cout<<center_x[0]<<std::endl;

    rclcpp::shutdown();
    return 0;
}