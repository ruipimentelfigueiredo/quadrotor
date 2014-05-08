#include <boost/bind.hpp>
#include <ros/node_handle.h>
#include <ros/callback_queue.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Eigen>
#include <hardware_interface/robot_hw.h>
#include <map>
#include <string>
#include <ros/callback_queue.h>
#include <hector_quadrotor_controller/pid.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <Eigen/StdVector>


using namespace hector_quadrotor_controller;

class Robot
{
public:

    PID x_;
    PID y_;
    PID z_;
    PID yaw_;

    /* Robot id */
    std::string id_;

    /* desired state within the virtual frame*/
    Eigen::Matrix<double,4,1, Eigen::DontAlign> desired_state_;

    /* current state within the virtual frame*/
    Eigen::Matrix<double,4,1, Eigen::DontAlign> current_state_;

    /* current state within the world frame*/
    Eigen::Matrix<double,4,1, Eigen::DontAlign> current_state_world_;

    /* current state within the world frame*/
    Eigen::Matrix<double,3,3, Eigen::DontAlign> rot_matrix_;


    /* ROS data */
    ros::NodeHandle n_;
    ros::NodeHandle n_priv;


    ros::Time previous;
    ros::Subscriber state_sub;
    ros::Publisher command_pub;

    bool first_time;

    Robot(ros::NodeHandle & n,
          std::string id,
          const Eigen::Matrix<double,4,1, Eigen::DontAlign> & desired_state);

    ~Robot();

    void stateCallback(const nav_msgs::OdometryConstPtr & state);

    void update(const Eigen::Matrix<double,4,1> &current_speed);

    void reset();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
