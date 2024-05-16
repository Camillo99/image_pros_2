#include "sensor_msgs/msg/point_cloud2.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include <pcl/point_types.h>
#include "rclcpp/rclcpp.hpp"

#include "image_pros_2/msg/hole_center_msg2.hpp"

#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

#include <Eigen/Dense>
#include <math.h> 
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

typedef pcl::PointXYZ PointT;

class PcTopic : public rclcpp::Node {

    public:
        ////////////////////////////////////
        //---Costructor---
        ////////////////////////////////////
        PcTopic();



    
    private:

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscription_;
        rclcpp::Subscription<image_pros_2::msg::HoleCenterMsg2>::SharedPtr center_subscription_;

        void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
        void center_callback(const image_pros_2::msg::HoleCenterMsg2::SharedPtr center_msg);

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_2_;
        // rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_3_;

        pcl::PointCloud<PointT>::Ptr raw_cloud_;
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
        

        // geometry_msgs::msg::Quaternion pclNormalToQuaternion(const pcl::Normal& pcl_normal);

};
