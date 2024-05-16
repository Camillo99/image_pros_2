#include "sensor_msgs/msg/point_cloud2.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include <pcl/point_types.h>
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

 

typedef pcl::PointXYZ PointT;


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

        std::cout<<("istance created default")<<std::endl;
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

        std::cout<<("istance created")<<std::endl;
      }


  //callback function definion
  void PointCloud2Dto3D::callback(const sensor_msgs::msg::PointCloud2::SharedPtr  cloud_msg) {
    
    std::cout<<("Callback method called")<<std::endl;
    point_cloud_received=true;
    //declare the writer --> save pcl image in pcl library format
    pcl::PCDWriter cloud_writer;
    std::string path="/home/camillo/zivid_ws/src/image_pros/point_clouds/";

    // convert the ROS PointCloud2 message to a PCL PCLPointCloud
    
    pcl::PointCloud<PointT>::Ptr pcl_pc2(new  pcl::PointCloud<PointT>);      //create a PCLPointCloud pointer
    pcl::fromROSMsg(*cloud_msg, *pcl_pc2) ;               //convert the pointcloud
    std::cout<<"source cloud points dimension "<<pcl_pc2->width * pcl_pc2->height<< std::endl;

    cloud_writer.write(path+std::string("cloud.pcd"),*pcl_pc2, false);

    

    //normal computation
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new  pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointT,pcl::Normal> normals_estimator;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    
    normals_estimator.setSearchMethod(tree);
    normals_estimator.setInputCloud(pcl_pc2);
    normals_estimator.setKSearch(10);
    normals_estimator.compute(*cloud_normals);

    std::cout<<("normal computation done !")<<std::endl;
    cloud_writer.write(path+std::string("cloud_norm.pcd"),*cloud_normals, false);


    //using indexes from the 2Dimage, extract the point correspoing to the center of each circle
    
    // Service call to HoleCenter service
    
    auto request = std::make_shared<image_pros::srv::HoleCenter::Request>();
    // request->flag=true;
    std::cout<<("service request created")<<std::endl;

    //if the service client is not available --> wait
/*    
    while (!hole_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
*/
    auto result = hole_client_->async_send_request(request);

    rclcpp::spin_some(this->get_node_base_interface()); // Spin until the service response arrives
        if (result.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
            RCLCPP_INFO(this->get_logger(), "Service call was successful");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }

    
    // std::cout<<("service request sent")<<std::endl;
    // int counter=0;
    // while (counter<10)
    // {
    //   /* code */
    //   counter++;
    //   rclcpp::sleep_for(std::chrono::milliseconds(1000));
    //   std::cout<<("Waiting")<<std::endl;
      
    // }
    
    // auto shared_this = shared_from_this();
    // if (rclcpp::spin_until_future_complete(shared_this, result) ==
    // rclcpp::FutureReturnCode::SUCCESS)
    // {
    //   std::cout<<"client answer receved !"<<std::endl;
    // }
    // else
    // {
    //   std::cout<<"waiting for the answer !"<<std::endl;
    // }

/*
    //wait for the result
    if (rclcpp::spin_until_future_complete(this, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "result from service arrived");
      //std::cout<<result.get()->hole_centers_x<std::endl;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service HoleCenter");
    }
*/
    //save the indexes vector
  // std::vector<short int> center_x = result.get()-> hole_centers_x;
    //std::vector<int> center_y = result.get()-> hole_centers_y;
    //std::cout<<result.get()->hole_centers_x<std::endl;
  //  std::cout<<center_x[1]<<std::endl;

    //std::cout<<result.get()->hole_centers_x<std::endl;


    std::vector<int> indicesToSelect = {1894096, 1049606, 1897448, 1414402, 657118, 1863846}; //from the python script
    pcl::PointCloud<PointT>::Ptr center_points (new  pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr center_normals(new  pcl::PointCloud<pcl::Normal>);


    // Iterate through each index in the list
    for (int index : indicesToSelect) {
        // Check if the index is within the range of the original cloud
        if (index >= 0 && index < pcl_pc2->size()) {
            // Add the point at the specified index to the selected cloud
            center_points->push_back(pcl_pc2->at(index));
        }
        // Check if the index is within the range of the original normal cloud
        if (index >= 0 && index < cloud_normals->size()) {
            // Add the point at the specified index to the selected cloud
            center_normals->push_back(cloud_normals->at(index));
        }
    }


    for (size_t i = 0; i < center_normals->size(); ++i) {
        const pcl::Normal& normal = center_normals->at(i);
        std::cout << "Normal " << i << ": "
                  << "x=" << normal.normal_x << ", "
                  << "y=" << normal.normal_y << ", "
                  << "z=" << normal.normal_z << std::endl;
    }

    for (size_t i = 0; i < center_points->size(); ++i) {
        const pcl::PointXYZ& point = center_points->at(i);
        std::cout << "Hole position " << i << ": "
                  << "x=" << point.x << ", "
                  << "y=" << point.y << ", "
                  << "z=" << point.z << std::endl;
    }


    // Set the frame ID of the new point cloud
    center_points->header.frame_id = "zivid_camera_frame";

    cloud_writer.write(path+std::string("center_points.pcd"),*center_points, false);
    cloud_writer.write(path+std::string("center_norm.pcd"),*center_normals, false);

    std::cout << "Number of points in the selected cloud: " << center_points->size() << std::endl;

    //convert from PCL PCLPointCloud2 to ROS PointCloud2 and pubblish on a new topic
    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl::toROSMsg(*center_points, output_cloud);
    
    publisher_->publish(output_cloud);
    std::cout<<("new point cloud published !")<<std::endl;
     
  }

  

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::cout << "Initializing ROS 2." << std::endl;
    //rclcpp::spin(std::make_shared<PointCloud2Dto3D>());     
    //rclcpp::Node::SharedPtr node1 = std::make_shared<PointCloud2Dto3D>();

    //PointCloud2Dto3D::SharedPtr node1 = std::make_shared<PointCloud2Dto3D>();
    auto node1 = std::make_shared<PointCloud2Dto3D>();

    // std::cout<<"flag at initialization "<<node1->isPointCloudReceived()<<std::endl;
    // rclcpp::executors::StaticSingleThreadedExecutor executor;
    // executor.add_node(node1);

    // int tmp_index = 0;
    // while(rclcpp::ok())
    // {
    //     executor.spin_once();
    //     //rclcpp::sleep_for(std::chrono::milliseconds(1000));
    //     std::cout << "Spinning. " << tmp_index << std::endl;
    //     tmp_index++;
    //     std::cout<<"flag  "<<node1->isPointCloudReceived()<<std::endl;
    // }
    rclcpp::spin(node1);
    rclcpp::shutdown();
    std::cout << "Shutting down ROS 2." << std::endl;
    return 0;
}
