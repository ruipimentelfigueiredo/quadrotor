#ifndef SWARM_FORMATION_CONTROL_H
#define SWARM_FORMATION_CONTROL_H

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
#include <32/bits/atomic_word.h>
#include <swarm_control/Robot.h>

using namespace hector_quadrotor_controller;




class SwarmFormationControl : public hardware_interface::RobotHW, public hardware_interface::HardwareInterface
{
public:

    SwarmFormationControl(ros::NodeHandle & n, std::vector<std::string> & robot_ids , const std::vector<Eigen::Matrix<double,4,1, Eigen::DontAlign> >& desired_formation);
    //void ~SwarmFormationControl(){};
    void updateCentroid();


    ros::CallbackQueue callback_queue_;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    std::vector<std::string> robot_ids_;
    std::vector<Eigen::Matrix<double,4,1, Eigen::DontAlign> > desired_formation_;
    std::map<std::string,boost::shared_ptr<Robot> > robot_;

    ros::Publisher swarm_centroid_pub;



};


#endif // SWARM_FORMATION_CONTROL_H
