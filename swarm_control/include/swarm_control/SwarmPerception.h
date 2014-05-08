#ifndef SWARM_PERCEPTION_H
#define SWARM_PERCEPTION_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

#include <map>

class SwarmPerception
{
public:

    SwarmPerception(ros::NodeHandle & n);
    //void ~SwarmPerception(){};


private:
    ros::NodeHandle n_;
    ros::NodeHandle n_priv_;
    tf::TransformListener listener_;
    laser_geometry::LaserProjection projector;
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    std::map<std::string,std::string> laser_topic_;
    std::map<std::string,std::string> laser_frame_;

    std::map<std::string,ros::Subscriber> laser_sub_;
    std::map<std::string,sensor_msgs::PointCloud> laser_readings_;

    ros::Publisher point_cloud_pub_;

};

#endif // SWARM_PERCEPTION_H
