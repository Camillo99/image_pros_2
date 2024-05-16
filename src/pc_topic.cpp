#include "image_pros_2/pc_topic.hpp"    //include the class PcTopic

  // default constructor
  PcTopic::PcTopic() 
      :  Node("pc_topic")
      {
        //define the subscription    
        pc_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                                  "/zivid/points/xyzrgba", 10,
                                  std::bind(&PcTopic::pc_callback, 
                                  this,
                                  std::placeholders::_1));
        center_subscription_ = this->create_subscription<image_pros_2::msg::HoleCenterMsg2>(
                            "/hole_center_2D", 10,
                            std::bind(&PcTopic::center_callback, 
                            this,
                            std::placeholders::_1));
        
        //define the publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point/center",10);
        publisher_2_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/hole_pose", 10);
        // publisher_3_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/hole_center_color", 10);

        raw_cloud_ = std::make_shared<pcl::PointCloud<PointT>>();
        cloud_normals_ = std::make_shared<pcl::PointCloud<pcl::Normal>>();



        std::cout<<("istance created default")<<std::endl;
      }



//callback function
//read the message from the /zivid/points/xyzrgba topic and
void PcTopic::pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr  cloud_msg) {
    RCLCPP_INFO(this->get_logger(), "Topic /zivid/points/xyzrgba from zivid received !");
    // convert the ROS PointCloud2 message to a PCL PCLPointCloud
    pcl::fromROSMsg(*cloud_msg, *raw_cloud_) ;
    RCLCPP_INFO(this->get_logger(), "Point Cloud saved !");

    RCLCPP_INFO(this->get_logger(), "Normal computation --> Started...");
    pcl::NormalEstimation<PointT,pcl::Normal> normals_estimator;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    
    normals_estimator.setSearchMethod(tree);
    normals_estimator.setInputCloud(raw_cloud_);
    normals_estimator.setKSearch(5);
    normals_estimator.compute(*cloud_normals_);
    RCLCPP_INFO(this->get_logger(), "Normal computation --> Done !");
    // RCLCPP_INFO(this->get_logger(), "Callback Exiting !");
}

//sub to the py topic
// using the indexes from the py topic, filter the pointCloud and the normCloud
//
void PcTopic::center_callback(const image_pros_2::msg::HoleCenterMsg2::SharedPtr center_msg){
    
    //set to false to hide printing
    bool print_flag = false;
    
    RCLCPP_INFO(this->get_logger(), "Topic /hole_center_2D from py node received !");

    //use the index from the topic to filters the pointcloud and the normcloud
    std::vector<int> center_index = center_msg->point_index;

    if(print_flag){
        // Printing the content of the array
        std::cout << "Content of center index array:" << std::endl;
        for (size_t i = 0; i < center_index.size(); ++i) {
            std::cout << "index[" << i << "] = " << center_index[i] << std::endl;
        }
    }

    pcl::PointCloud<PointT>::Ptr center_points (new  pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr center_normals(new  pcl::PointCloud<pcl::Normal>);

    // filter the pointcloud and the normcloud
    for (int index : center_index) {
        // Check if the index is within the range of the original cloud
        if (index >= 0 && index < raw_cloud_->size()) {
            // Add the point at the specified index to the selected cloud
            center_points->push_back(raw_cloud_->at(index));
        }
        // Check if the index is within the range of the original normal cloud
        if (index >= 0 && index < cloud_normals_->size()) {
            // Add the point at the specified index to the selected cloud
            center_normals->push_back(cloud_normals_->at(index));
        }
    }
    
    if(print_flag){
        for (size_t i = 0; i < center_points->size(); ++i) {
            const pcl::PointXYZ& point = center_points->at(i);
            std::cout << "Hole position " << i << ": "
                    << "x=" << point.x << ", "
                    << "y=" << point.y << ", "
                    << "z=" << point.z << std::endl;
        }

        for (size_t i = 0; i < center_normals->size(); ++i) {
            const pcl::Normal& normal = center_normals->at(i);
            std::cout << "Normal " << i << ": "
                    << "x=" << normal.normal_x << ", "
                    << "y=" << normal.normal_y << ", "
                    << "z=" << normal.normal_z << std::endl;
        }
    }


    //pubblish on a pointCloud2 topic the position of the centerhole
    // Set the frame ID of the new point cloud
    center_points->header.frame_id = "zivid_camera_frame";
    //convert from PCL PCLPointCloud to ROS PointCloud2 and pubblish on a new topic
    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl::toROSMsg(*center_points, output_cloud);
    
    publisher_->publish(output_cloud);
    RCLCPP_INFO(this->get_logger(), "Publishing center points as PointCloud !");

    
    //create a pose array from geometry_msgs and add all the pose of the holes
    //auto poseArrayMsg = std::make_shared<geometry_msgs::msg::PoseArray>();
    geometry_msgs::msg::PoseArray poseArrayMsg;

    // Set frame ID for the PoseArray message
    poseArrayMsg.header.frame_id = "zivid_camera_frame";
    for (size_t i = 0; i < center_points->size(); ++i) {
        geometry_msgs::msg::Pose pose;
        const pcl::PointXYZ& point = center_points->at(i);
        const pcl::Normal& normal = center_normals->at(i);

        pose.position.x = point.x;
        pose.position.y = point.y;
        pose.position.z = point.z;

        //convert from normal vector to quaternion representation
        // geometry_msgs::msg::Quaternion normal_quat = pclNormalToQuaternion(normal);

        // pose.orientation.x = normal_quat.x;
        // pose.orientation.y = normal_quat.y;
        // pose.orientation.z = normal_quat.z;
        // pose.orientation.w = normal_quat.w;
        pose.orientation.x = normal.normal_x;
        pose.orientation.y = normal.normal_y;
        pose.orientation.z = normal.normal_z;
        pose.orientation.w = 0.0;

        poseArrayMsg.poses.push_back(pose);
    }
    // Publish the PoseArray message
    publisher_2_->publish(poseArrayMsg);

    // std::cout<<"--------------done1-------------------"<< std::endl;

    // //get color from py_node and create a new message with both pose and color
    // colorArraymsg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    // colorArraymsg.layout.dim[0].label = "height";
    // colorArraymsg.layout.dim[0].size = HEIGHT;
    // colorArraymsg.layout.dim[0].stride = HEIGHT*WIDTH;
    // // colorArraymsg.layout.dim[1].label = "width";
    // // colorArraymsg.layout.dim[1].size = WIDTH;
    // // colorArraymsg.layout.dim[1].stride = WIDTH;
    // // colorArraymsg.layout.data_offset = 0;

    // // std::vector<float> dataArray(myArray, myArray + center_points->size());
    // colorArraymsg.data= vec1;
    // // std::cout<<vec1<< std::endl;
    // publisher_3_->publish(colorArraymsg);
    // std::cout<<"--------------done2-------------------"<< std::endl;

}














int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PcTopic>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
