#ifndef SERVER_H
#define SERVER_H

//ROS2 Headers
#include "rclcpp/rclcpp.hpp"
// //Autoware Headers
// #include "autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp"
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <geometry_msgs/msg/quaternion.hpp>
// #include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/buffer.h>
// #include <autoware_auto_vehicle_msgs/msg/vehicle_control_command.hpp>
//OpenMP thread header
#include <omp.h>
//Eigen Headers
// #include <Eigen/Dense>
// //PCL Headers
// #include <pcl/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/voxel_grid.h>
//std C++ Headers


#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>
#include <string>

using namespace std::chrono_literals;

#define NUMBER_OF_ARGUMENTS 1
#define PORTADDR 9009

class TCPServer : public rclcpp::Node
{
  public:
    TCPServer(char* ip_port);
    ~TCPServer();

  private:
    void socketConnect();
    void transfer();


    int ServerSocket, ClientSocket;
    struct sockaddr_in SocketAddress;
    int addrLen = sizeof(SocketAddress);


    //ROS topic publisher
    // rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VehicleKinematicState>::SharedPtr publisher_;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_publisher_front;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_publisher_rear;
    // //ROS topic subsriber
    // rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VehicleControlCommand>::SharedPtr subscriber_;
    // rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;
    // //Autoware message_initialization
    // autoware_auto_vehicle_msgs::msg::VehicleKinematicState msg_;
    // sensor_msgs::msg::PointCloud2 pointcloud_msg_front;
    // sensor_msgs::msg::PointCloud2 pointcloud_msg_rear;
    // // Declare all required variables
    // uint16_t port;
    // uint8_t sensor_id1;
    // uint8_t sensor_id2;
    // std::string ip;
    // std::string rec_rep_mode;
};

#endif  // DSPACE_AUTOWARE
