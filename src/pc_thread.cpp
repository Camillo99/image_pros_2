#include "sensor_msgs/msg/point_cloud2.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include <pcl/point_types.h>
#include "pcl/visualization/cloud_viewer.h"
#include "image_pros/pc_thread.hpp"    //include the class PcThread
#include <pcl/filters/voxel_grid.h>
#include "rclcpp/rclcpp.hpp"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <chrono>
using namespace std::chrono_literals;


//default constructor
PcThread::PcThread()
    :   Node("pc_thread")
    {
        //define the callback group
        topic_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        //define the topic subscription
        size_t queue_size = 10;
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "zivid/points/xyzrgba",
            queue_size,
            std::bind(&PcThread::sub_callback, this, std::placeholders::_1));
        
        //define the service client
        hole_client_ptr_ = this->create_client<image_pros::srv::HoleCenter>("hole_center", 
            rmw_qos_profile_services_default,
            client_cb_group_);
        
        //define the publisher
        //TODO

        std::cout<<("istance created default")<<std::endl;
    }

void PcThread::sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg){
    std::cout<<("Inside the sub callback")<<std::endl;

    //convert pc

    //TODO
    rclcpp::sleep_for(std::chrono::milliseconds(5000));
    //call service

    RCLCPP_INFO(this->get_logger(), "Sending request");
    auto request = std::make_shared<image_pros::srv::HoleCenter::Request>();
    auto result_future = hole_client_ptr_->async_send_request(request);
    rclcpp::sleep_for(std::chrono::milliseconds(5000));
    std::future_status status = result_future.wait_for(10s);  // timeout to guarantee a graceful finish
    if (status == std::future_status::ready) {
        RCLCPP_INFO(this->get_logger(), "Received response");
    }

    //save the indexes vector
    std::vector<short int> center_x = result_future.get()-> hole_centers_x;
    std::vector<short int> center_y = result_future.get()-> hole_centers_y;
    //std::cout<<result.get()->hole_centers_x<std::endl;
    //std::cout<<center_x[1]<<std::endl;


}






/////////////////////////
//--main/--//////////////
/////////////////////////

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto my_node = std::make_shared<PcThread>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(my_node);

    RCLCPP_INFO(my_node->get_logger(), "Starting client node, shut down with CTRL-C");
    executor.spin();
    RCLCPP_INFO(my_node->get_logger(), "Keyboard interrupt, shutting down.\n");

    rclcpp::shutdown();
    return 0;
}