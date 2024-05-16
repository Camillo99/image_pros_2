#include "sensor_msgs/msg/point_cloud2.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "pcl/visualization/cloud_viewer.h"
#include "image_pros/pointcloud_2Dto3D.hpp"    //include the class PointCloud2Dto3D
#include <pcl/filters/voxel_grid.h>
#include "rclcpp/rclcpp.hpp"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>





// default constructor
PointCloud2Dto3D::PointCloud2Dto3D() 
    :  Node("pointcloud_2Dto3D")
    {
        //define the subscription    
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                                    "/zivid/points/xyzrgba", 10,
                                    std::bind(&PointCloud2Dto3D::callback, 
                                    this,
                                    std::placeholders::_1));
        //define the publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("Point_process",10);
        publisher_2_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("Hole_pose", 10);
        hole_client_ = this->create_client<image_pros::srv::HoleCenter>("hole_center");
        point_cloud_received=false;
        raw_cloud_ = std::make_shared<pcl::PointCloud<PointT>>();
        cloud_normals_ = std::make_shared<pcl::PointCloud<pcl::Normal>>();

        RCLCPP_INFO(this->get_logger(), "Instance of PointCloud2Dto3D created !");
    }

//constructor with parameter (is it useful ?)
PointCloud2Dto3D::PointCloud2Dto3D(std::string node_name_) 
    :  Node(node_name_) 
    {
        //define the subscription
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                                "/zivid/points/xyzrgba", 10,
                                std::bind(&PointCloud2Dto3D::callback, 
                                this,
                                std::placeholders::_1));
        //define the publisher                       
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("Point_process",10);
        publisher_2_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("Hole_pose", 10);
        hole_client_ = this->create_client<image_pros::srv::HoleCenter>("hole_center");
        point_cloud_received=false;
        raw_cloud_ = std::make_shared<pcl::PointCloud<PointT>>();
        cloud_normals_ = std::make_shared<pcl::PointCloud<pcl::Normal>>();

        std::cout<<("istance created")<<std::endl;
    }


//callback function
//read the message from the /zivid/points/xyzrgba topic and
//save it in the class variable raw_cloud_
void PointCloud2Dto3D::callback(const sensor_msgs::msg::PointCloud2::SharedPtr  cloud_msg) {
    RCLCPP_INFO(this->get_logger(), "Topic received !");
    // convert the ROS PointCloud2 message to a PCL PCLPointCloud
    pcl::fromROSMsg(*cloud_msg, *raw_cloud_) ;
    RCLCPP_INFO(this->get_logger(), "Point Cloud saved !");
    point_cloud_received=true;
    RCLCPP_INFO(this->get_logger(), "Callback Exiting !");
}

//compute the normal surface to each point in the cloud
   //save the norm cloud in class variable --> cloud_normals_
void PointCloud2Dto3D::pc_processing(){

    RCLCPP_INFO(this->get_logger(), "Normal computation --> Started...");
    pcl::NormalEstimation<PointT,pcl::Normal> normals_estimator;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    
    normals_estimator.setSearchMethod(tree);
    normals_estimator.setInputCloud(raw_cloud_);
    normals_estimator.setKSearch(10);
    normals_estimator.compute(*cloud_normals_);
    RCLCPP_INFO(this->get_logger(), "Normal computation --> Done !");
}

void PointCloud2Dto3D::hole_service_call(){

    auto request = std::make_shared<image_pros::srv::HoleCenter::Request>();
     auto response_received = std::make_shared<std::promise<bool>>();
    //auto result = hole_client_->async_send_request(request);
    // Wait for the result.
    // auto shared_this = shared_from_this();
    // std::cout<<"here !!"<<std::endl;
    // if (rclcpp::spin_until_future_complete(shared_this, result) ==
    //     rclcpp::FutureReturnCode::SUCCESS)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Service called succesfully");
    // } else {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to call the service");
    // }
    // std::vector<short int> center_x = result.get()-> hole_centers_x;
    // std::vector<short int> center_y = result.get()-> hole_centers_y;
    // std::cout<<center_x[1]<<std::endl;

    // Asynchronously send the service request
    auto result = hole_client_->async_send_request(request, [this, response_received](rclcpp::Client<image_pros::srv::HoleCenter>::SharedFuture future) {
        auto response = future.get(); // Get the response
        if (response) {
            std::vector<short int> center_x = response->hole_centers_x;
            std::vector<short int> center_y = response->hole_centers_y;
            std::cout << center_x[1] << std::endl;
            response_received->set_value(true); // Set the promise value to true if response received successfully
        } else {
            response_received->set_value(false); // Set the promise value to false if failed to receive response
        }
    });
    
    // Wait for the promise to be fulfilled
    auto future = response_received->get_future();
    if (future.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
        RCLCPP_ERROR(get_logger(), "Timed out waiting for service response");
        // Handle timeout error
    } else {
        bool response_success = future.get(); // Get the value of the promise
        if (response_success) {
            RCLCPP_INFO(get_logger(), "Service called successfully");
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to call the service");
            // Handle service call failure
        }
    }

}








int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::cout << "Initializing ROS 2." << std::endl;

    auto node1 = std::make_shared<PointCloud2Dto3D>();

    std::cout<<"flag at initialization "<<node1->isPointCloudReceived()<<std::endl;
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(node1);

    int tmp_index = 0;
    while(rclcpp::ok())
    {
        if(!node1->isPointCloudReceived()){
            //wait for the topic
            std::cout<< "waiting for a topic ..."<<std::endl;
            executor.spin_once();
            
        }
        else{
            //topic is received
            std::cout<< "inside else condition"<<std::endl;
            //process the pointcloud --> normal computation
            node1->pc_processing();
            //call the service
            node1->hole_service_call();
            //publish the result

            //set the flag so the node is again ready to receive a new pointcloud 
            node1->setPointCloudReceived(false);
        }
        std::cout << "Spinning. " << tmp_index << std::endl;
        tmp_index++;
        std::cout<<"flag  "<<node1->isPointCloudReceived()<<std::endl;
    }
    rclcpp::shutdown();
    std::cout << "Shutting down ROS 2." << std::endl;
    return 0;
}
