#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <actionlib/server/simple_action_server.h>
#include <swarm_msgs/SwarmFormationAction.h>
#include <tf/transform_broadcaster.h>

class SwarmFormationAction
{
public:
    
    SwarmFormationAction(std::string name) :
        as_(nh_, name, false),
        action_name_(name)
    {
        //register the goal and feeback callbacks
        as_.registerGoalCallback(boost::bind(&SwarmFormationAction::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&SwarmFormationAction::preemptCB, this));

        //subscribe to the data topic of interest
        sub_ = nh_.subscribe("/swarm_centroid", 1, &SwarmFormationAction::analysisCB, this);
        as_.start();
    }

    ~SwarmFormationAction(void)
    {
    }

    void goalCB()
    {
        // accept the new goal
        pose_goal_ = as_.acceptNewGoal()->pose;
        //precision_goal_ = as_.acceptNewGoal()->precision;
    }

    void preemptCB()
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
    }

    void analysisCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        // make sure that the action hasn't been canceled
        if (!as_.isActive())
            return;


        static tf::TransformBroadcaster br;

        tf::Quaternion q(pose_goal_.pose.orientation.x,
                         pose_goal_.pose.orientation.y,
                         pose_goal_.pose.orientation.z,
                         pose_goal_.pose.orientation.w);
        transform.setRotation(q);
        transform.setOrigin( tf::Vector3(pose_goal_.pose.position.x,
                                         pose_goal_.pose.position.y,
                                         pose_goal_.pose.position.z) );

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "swarm_goal"));


        feedback_.error.pose.position.x=pose_goal_.pose.position.x-msg->pose.position.x;
        feedback_.error.pose.position.y=pose_goal_.pose.position.y-msg->pose.position.y;
        feedback_.error.pose.position.z=pose_goal_.pose.position.z-msg->pose.position.z;
        feedback_.error.pose.orientation.x=pose_goal_.pose.orientation.x-msg->pose.orientation.x;
        feedback_.error.pose.orientation.y=pose_goal_.pose.orientation.y-msg->pose.orientation.y;
        feedback_.error.pose.orientation.z=pose_goal_.pose.orientation.z-msg->pose.orientation.z;
        feedback_.error.pose.orientation.w=pose_goal_.pose.orientation.w-msg->pose.orientation.w;
        as_.publishFeedback(feedback_);

        double error_translation=sqrt((feedback_.error.pose.position.x*
                                       feedback_.error.pose.position.x)+
                                      (feedback_.error.pose.position.y*
                                       feedback_.error.pose.position.y)+
                                      (feedback_.error.pose.position.z*
                                       feedback_.error.pose.position.z)
                                      );

        // Check if goal was reached
        precision_goal_=0.8;
        if(error_translation < precision_goal_)
        {

            /*result_.mean = feedback_.mean;
            result_.std_dev = feedback_.std_dev;

            if(result_.mean < 5.0)
            {
                ROS_INFO("%s: Aborted", action_name_.c_str());
                //set the action state to aborted
                as_.setAborted(result_);
            }*/
//            else
            {
                ROS_INFO("%s: Succeeded", action_name_.c_str());
                // set the action state to succeeded
                as_.setSucceeded(result_);
            }
        }
    }

protected:
    tf::Transform transform;

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<swarm_msgs::SwarmFormationAction> as_;
    std::string action_name_;
    geometry_msgs::PoseStamped  pose_goal_;
    double precision_goal_;
    float sum_, sum_sq_;
    swarm_msgs::SwarmFormationFeedback feedback_;
    swarm_msgs::SwarmFormationResult result_;
    ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "swarm_control_action");

    SwarmFormationAction swarm_control_server(ros::this_node::getName());
    ros::spin();

    return 0;
}
