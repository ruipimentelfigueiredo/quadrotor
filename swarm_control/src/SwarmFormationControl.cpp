#include <swarm_control/SwarmFormationControl.h>

SwarmFormationControl::SwarmFormationControl(ros::NodeHandle & n,
                                             std::vector<std::string> & robot_ids ,
                                             const std::vector<Eigen::Matrix<double,4,1, Eigen::DontAlign> >& desired_formation):
    robot_ids_(robot_ids), desired_formation_(desired_formation)
{
    // Initialize robots
    for(int i=0;i<robot_ids_.size();++i)
    {
        boost::shared_ptr<Robot> robot(new Robot(n,robot_ids_[i],desired_formation_[i]));
        robot_.insert( std::pair<std::string,boost::shared_ptr<Robot> >(robot_ids_[i],robot));
    }

    swarm_centroid_pub=n.advertise<geometry_msgs::PoseStamped>("swarm_centroid",10);
}

/* Update formation centroid state */
void SwarmFormationControl::updateCentroid()
{
    double x,y,z;
    x=y=z=0.0;
    std::map<std::string, boost::shared_ptr<Robot> >::iterator it_robot;
    for(it_robot = robot_.begin(); it_robot != robot_.end(); ++it_robot)
    {
        x+=it_robot->second->current_state_world_(0,0);
        y+=it_robot->second->current_state_world_(1,0);
        z+=it_robot->second->current_state_world_(2,0);
    }
    x/=robot_.size();
    y/=robot_.size();
    z/=robot_.size();
    std::string centroid_name="swarm_centroid";
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x, y, z) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", centroid_name));

    geometry_msgs::PoseStamped centroid_msg;
    centroid_msg.pose.orientation.x=q.getX();
    centroid_msg.pose.orientation.y=q.getY();
    centroid_msg.pose.orientation.z=q.getZ();
    centroid_msg.pose.orientation.w=q.getW();
    centroid_msg.pose.position.x=x;
    centroid_msg.pose.position.y=y;
    centroid_msg.pose.position.z=z;


    swarm_centroid_pub.publish(centroid_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_commander");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");


    ////////////////////////////
    // Load object part types //
    ////////////////////////////



    std::vector<std::string> robot_ids;
    std::vector<Eigen::Matrix<double,4,1, Eigen::DontAlign> > desired_formation;

    XmlRpc::XmlRpcValue swarm;
    n_priv.getParam("swarm", swarm);
    ROS_INFO_STREAM("Loading swarm of " << swarm.size() << " robots.");
    std::map<std::string, XmlRpc::XmlRpcValue>::iterator i;
    for (i = swarm.begin(); i != swarm.end(); i++)
    {
        robot_ids.push_back(i->first);
        Eigen::Matrix<double,4,1, Eigen::DontAlign> pose;
        double x=i->second["pose"]["x"]["value"];
        double y=i->second["pose"]["y"]["value"];
        double z=i->second["pose"]["z"]["value"];
        double yaw=i->second["pose"]["yaw"]["value"];
        pose << x, y,z, yaw;
        desired_formation.push_back(pose);
    }

    SwarmFormationControl swarm_formation(n,robot_ids,desired_formation);

    ros::Publisher goal_pub=n.advertise<geometry_msgs::PoseStamped>("swarm_goal",10);
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.pose.position.x=0.0;
    goal_msg.pose.position.y=0.0;
    goal_msg.pose.position.z=1.0;


    //    static tf::TransformBroadcaster brr;
    //    tf::Transform transform;
    //    int increment=1;
    //    tf::Quaternion q;
    //    q.setRPY(0, 0, 0);
    //    transform.setRotation(q);
    //    transform.setOrigin( tf::Vector3(0.1, 0.1, 1.0+0.01) );


    ros::AsyncSpinner spinner(4);
    spinner.start();


    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        //        ++increment;

        //        transform.setOrigin( tf::Vector3(1.0, 1.0, 1.0) );
        //        transform.setOrigin( tf::Vector3(0.1+increment*0.001, 0.1+increment*0.001, 1.0+increment*0.001) );

        //        q.setRPY(0, 0, 0+increment*0.001);
        //        transform.setRotation(q);
        //        brr.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "swarm_goal"));

        swarm_formation.updateCentroid();
        loop_rate.sleep();
        goal_msg.pose.position.x+=0.01;

        goal_msg.pose.position.z+=0.001;
        goal_pub.publish(goal_msg);

    }
    spinner.stop();

    return 0;
}

