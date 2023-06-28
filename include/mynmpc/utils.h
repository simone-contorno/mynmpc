#ifndef UTILS_H
#define UTILS_H

/********************************************************************************/
/*********************************** Include ************************************/
/********************************************************************************/

/* ROS2 C++ */
#include <rclcpp/rclcpp.hpp>

/* Messages */
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

/* Other libraries */
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <urdf/model.h>
#include <assert.h>
#include <stdlib.h>

/********************************************************************************/
/************************************ Using *************************************/
/********************************************************************************/

/* Eigen */
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* Types */
using nav_msgs::msg::Path;
using SetParametersResult = rcl_interfaces::msg::SetParametersResult_<std::allocator<void>>;
using PubFloat64 = std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64, std::allocator<void>>>;
using PubTwist = std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist, std::allocator<void>>>;
using PubFloat32MultiArray = std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32MultiArray, std::allocator<void>>>;

/* Namespaces */
using namespace std;

/********************************************************************************/
/********************************** Functions ***********************************/
/********************************************************************************/

void normalizeAngle(double &angle); 
MatrixXd computeBox(VectorXd pose, double l, double w, double l_down, double l_right, size_t points);
std::tuple<size_t, size_t> closestBoxPoints(MatrixXd box_1, MatrixXd box_2);
Path optimPath(MatrixXd x, const rclcpp::Time &now);

#endif
