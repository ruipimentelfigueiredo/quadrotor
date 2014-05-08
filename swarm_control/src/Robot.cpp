#include <swarm_control/Robot.h>

Robot::Robot(ros::NodeHandle & n, std::string id, const Eigen::Matrix<double,4,1, Eigen::DontAlign> & desired_state) : n_(n),id_(id),  first_time(true), desired_state_(desired_state)
{
    n_priv=ros::NodeHandle("~/swarm/"+id+"/pose");
    std::string sub_topic_name=id+"/ground_truth/state";
    state_sub=n_.subscribe<nav_msgs::Odometry> (sub_topic_name,10,boost::bind(&Robot::stateCallback, this,_1));
    command_pub=n_.advertise<geometry_msgs::Twist>(id+"/cmd_vel",10);

    x_.init(ros::NodeHandle(n_priv, "x"));
//    std::cout << "id:" << id << " pid.p:"<< x_.parameters_.k_p << std::endl;
    x_.parameters_.enabled=true;


    y_.init(ros::NodeHandle(n_priv, "y"));
    y_.parameters_.enabled=true;


    z_.init(ros::NodeHandle(n_priv, "z"));
    z_.parameters_.enabled=true;


    yaw_.init(ros::NodeHandle(n_priv, "yaw"));
    yaw_.parameters_.enabled=true;

    previous=ros::Time::now();
}

Robot::~Robot()
{
    geometry_msgs::Twist twist_msg;
    command_pub.publish(twist_msg);
}


void Robot::stateCallback(const nav_msgs::OdometryConstPtr & state)
{
    ////////////////////////////////////////
    // Robot pose relative to world frame //
    ////////////////////////////////////////

    geometry_msgs::PoseStamped current_pose_world;
    current_pose_world.pose=state->pose.pose;
    current_pose_world.header.frame_id="world";

    double roll, pitch, yaw;
    tf::Quaternion q(current_pose_world.pose.orientation.x,
                     current_pose_world.pose.orientation.y,
                     current_pose_world.pose.orientation.z,
                     current_pose_world.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll,pitch,yaw);

    // WE DON'T CARE ABOUT ROLL AND PITCH

    current_state_world_(0,0)=current_pose_world.pose.position.x;
    current_state_world_(1,0)=current_pose_world.pose.position.y;
    current_state_world_(2,0)=current_pose_world.pose.position.z;
    current_state_world_(3,0)=yaw;

    /////////////////////////////////////////////////////
    // Get pose relative to virtual frame (goal frame) //
    /////////////////////////////////////////////////////

    tf::TransformListener listener;

    geometry_msgs::PoseStamped current_pose_virtual;
    //current_pose_virtual=current_pose_world;
    try
    {
        listener.waitForTransform("world", "/swarm_goal", ros::Time(0), ros::Duration(1.0));
        listener.transformPose("/swarm_goal", ros::Time(0), current_pose_world
                               ,"world",current_pose_virtual);
    }
    catch (tf::TransformException ex)
    {
        ROS_INFO("%s",ex.what());
        geometry_msgs::Twist twist_msg;
        command_pub.publish(twist_msg);
        return;
    }

    rot_matrix_=Eigen::Quaterniond(current_pose_virtual.pose.orientation.w,
                                   current_pose_virtual.pose.orientation.x,
                                   current_pose_virtual.pose.orientation.y,
                                   current_pose_virtual.pose.orientation.z).matrix();

    tf::Quaternion q1(current_pose_virtual.pose.orientation.x,
                      current_pose_virtual.pose.orientation.y,
                      current_pose_virtual.pose.orientation.z,
                      current_pose_virtual.pose.orientation.w);



    tf::Matrix3x3(q1).getRPY(roll,pitch,yaw);

    current_state_(0,0)=current_pose_virtual.pose.position.x;
    current_state_(1,0)=current_pose_virtual.pose.position.y;
    current_state_(2,0)=current_pose_virtual.pose.position.z;
    current_state_(3,0)=yaw;

    //        std::cout <<id_<<":virtual pose:"<<current_state_.transpose()<< std::endl;
    //        std::cout <<id_<<":desired pose:"<<desired_state_.transpose()<< std::endl;


    Eigen::Matrix<double,4,1> velocity_vec;
    velocity_vec(0,0)=state->twist.twist.linear.x;
    velocity_vec(1,0)=state->twist.twist.linear.y;
    velocity_vec(2,0)=state->twist.twist.linear.z;
    velocity_vec(3,0)=state->twist.twist.angular.z;

    Eigen::Matrix<double,3,1> velocity_vec_aux;
    velocity_vec_aux <<velocity_vec(0,0),velocity_vec(1,0),velocity_vec(2,0);

    Eigen::Matrix<double,3,1> velocity_vec_virtual_frame;
    velocity_vec_virtual_frame=rot_matrix_.inverse()*velocity_vec_aux;

    velocity_vec << velocity_vec_virtual_frame, velocity_vec(3,0);

    update(velocity_vec);
}

void Robot::update(const Eigen::Matrix<double,4,1> &current_speed)
{
    ros::Time now=ros::Time::now();
    ros::Duration period=now-previous;
    previous=now;
    if(first_time)
    {
        first_time=false;
        return;
    }
    Eigen::Matrix<double,3,1> translation_error;
    translation_error(0,0)=desired_state_(0,0)-current_state_(0,0);
    translation_error(1,0)=desired_state_(1,0)-current_state_(1,0);
    translation_error(2,0)=desired_state_(2,0)-current_state_(2,0);


    Eigen::Matrix<double,3,1> rotation_error;
    rotation_error(0,0)=0.0;
    rotation_error(1,0)=0.0;
    rotation_error(2,0)=desired_state_(3,0)-current_state_(3,0);


    translation_error=rot_matrix_.inverse()*translation_error;
    rotation_error=rot_matrix_.inverse()*rotation_error;


    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x=x_.update(translation_error(0,0), current_speed(0,0), period);
    twist_msg.linear.y=y_.update(translation_error(1,0), current_speed(1,0), period);
    twist_msg.linear.z=z_.update(translation_error(2,0), current_speed(2,0), period);
    twist_msg.angular.z=yaw_.update(rotation_error(2,0), current_speed(3,0), period);
    //        std::cout << "angular out:"<<twist_msg.angular.z << std::endl;

    //twist_msg.angular.z=0.0;

    command_pub.publish(twist_msg);

}

void Robot::reset()
{
    x_.reset();
    y_.reset();
    z_.reset();
    yaw_.reset();
}



