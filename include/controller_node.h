#ifndef __controller_node__
#define __controller_node__
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <iostream>
#include <controller_node.h>
#include <my_mpc/mympc.h>
#include "mavros_msgs/AttitudeTarget.h"
#include "geometry_msgs/PoseStamped.h"

//#include <mav_msgs/RollPitchYawrateThrust.h>
// #include <mav_msgs/RollPitchYawrateThrust.h>
class ControllerNode
{
public:
    // Constructor
    ControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_param)
    :nh_(nh), nh_param_(nh_param)
    {
        std::cout<<"[controller_node.h] : Running controller node!"<<std::endl;
        pub_              = nh_.advertise<mavros_msgs::AttitudeTarget>("/scout/mavros/setpoint_raw/attitude", 100);
        pub_marker_       = nh_.advertise<visualization_msgs::Marker>("/scout/visualization_marker", 10);
        pub_direction_    = nh_.advertise<geometry_msgs::PoseStamped>("/scout/debug_pose", 10);

        //        pub_debug_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust>("/firefly/command/roll_pitch_yawrate_thrust",100);

        sub_              = nh_.subscribe("/scout/mavros/local_position/odom", 100, &ControllerNode::OdometryCallback, this);
        sub_goal_         = nh_.subscribe("/scout/goal_pose", 1, &ControllerNode::GoalCallbackByPoseStamped, this);
        sub_goal_mission_ = nh_.subscribe("/scout/goal_mission", 1, &ControllerNode::GoalCallbackByMission, this); // this is for debug
        sub_goalaction_   = nh_.subscribe("/scout/GoalAction",1,&ControllerNode::GoalActionCallback,this); //this is for connection with mission_planner
        
        // sub_ = nh_.subscribe("/firefly/ground_truth/odometry", 100, &ControllerNode::OdometryCallback, this);

        controller_ = new mympc::ModelPredictiveController(nh, nh_param);
        
        attitude_target.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | 
                                    mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                                    mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE ;
        attitude_target.orientation.x = 0;
        attitude_target.orientation.y = 0;
        attitude_target.orientation.z = 0;
        attitude_target.orientation.w = 1;

        attitude_target.thrust = 0;
        
    }
    // Odometry Callback
    void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void GoalCallbackByPoseStamped(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void GoalCallbackByMission(const std_msgs::Float32MultiArray& msg);
    void PublishCommand();
    void GoalActionCallback(const std_msgs::Float32MultiArray& goalaciton);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_param_;
    ros::Publisher  pub_;
    ros::Publisher  pub_debug_;
    ros::Publisher  pub_marker_;
    ros::Publisher  pub_direction_;
    ros::Subscriber sub_;
    ros::Subscriber sub_goal_;
    ros::Subscriber sub_goal_mission_;
    ros::Subscriber sub_goalaction_;
    mympc::ModelPredictiveController* controller_;

    //command
    Eigen::Vector4d* ref_attitude_thrust = new Eigen::Vector4d;
    mavros_msgs::AttitudeTarget attitude_target;
};


void ControllerNode::GoalActionCallback(const std_msgs::Float32MultiArray& goalaction)
{
    double goal_x   = goalaction.data[1];
    double goal_y   = goalaction.data[2];
    double goal_z   = goalaction.data[3];
    double goal_yaw = goalaction.data[4]; // radian

    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = goal_x;
    goal.pose.position.y = goal_y;
    goal.pose.position.z = goal_z; 

    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(goal_yaw, Eigen::Vector3d::UnitZ());
    
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();

    controller_->setGoal(goal);

    // TODO:: implement velocity option
    //controller_->setVelocity(goalaction.data[5]);

}
void ControllerNode::GoalCallbackByMission(const std_msgs::Float32MultiArray& msg)
{
    double goal_x   = msg.data[0];
    double goal_y   = msg.data[1];
    double goal_z   = msg.data[2];
    double goal_yaw = msg.data[3] / 180 * M_PI;
    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = goal_x;
    goal.pose.position.y = goal_y;
    goal.pose.position.z = goal_z; 
    
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(goal_yaw, Eigen::Vector3d::UnitZ());
    
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();

    // std::cout<<
    // "goal_x : "<<goal_x<<std::endl<<
    // "goal_y : "<<goal_y<<std::endl<<
    // "goal_z : "<<goal_z<<std::endl<<
    // "goal_yaw : "<<goal_yaw<<std::endl;
    controller_->setGoal(goal);
    // PublishCommand();
}
void ControllerNode::GoalCallbackByPoseStamped(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // std::cout<<"[controller_node.h] : run goal callback!"<<std::endl;
    controller_->setGoal(*msg);
    // PublishCommand();
}
void ControllerNode::OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // std::cout<<"[controller_node.h] : run odometry callback!"<<std::endl;
    controller_->setOdometry(*msg);
    PublishCommand();
}

void ControllerNode::PublishCommand()
{
    // std::cout<<"publish command"<<std::endl;
    static int publish_count = 0;
    
    controller_->calculateRollPitchYawrateThrustCommand(ref_attitude_thrust);
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd((*ref_attitude_thrust)[0], Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd((*ref_attitude_thrust)[1], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd((*ref_attitude_thrust)[2], Eigen::Vector3d::UnitZ());
    
    tf::Quaternion quat_from_rpy;
    quat_from_rpy.setRPY((*ref_attitude_thrust)[0], (*ref_attitude_thrust)[1],(*ref_attitude_thrust)[2]);
    // attitude_target.orientation.x = q.x();
    // attitude_target.orientation.y = q.y();
    // attitude_target.orientation.z = q.z();
    // attitude_target.orientation.w = q.w();

    attitude_target.orientation.x = quat_from_rpy.x();
    attitude_target.orientation.y = quat_from_rpy.y();
    attitude_target.orientation.z = quat_from_rpy.z();
    attitude_target.orientation.w = quat_from_rpy.w();

    // attitude_target.orientation.x = attitudetarget_orientation.x();
    // attitude_target.orientation.y = attitudetarget_orientation.y();
    // attitude_ta[controller_node.h]============="<<std::endl;
    

    Eigen::Vector3d euler_angles;

    euler_angles = q.toRotationMatrix().eulerAngles(0,1,2);

    attitude_target.body_rate.x = 0;
    attitude_target.body_rate.y = 0;
    attitude_target.body_rate.z = 0;


    // double throttle = (*ref_attitude_thrust)[3] * a + b;
    double throttle = (*ref_attitude_thrust)[3] / 1.50 / 9.8 * 0.707; // 2d_platform
    // double throttle = (*ref_attitude_thrust)[3] / 2.30 / 9.8 * 0.66; // gazebo
    
    if (throttle >= 0.80)
    {throttle = 0.80;}
    
    attitude_target.thrust = (throttle);
    

    // PUBLISH!!!
    pub_.publish(attitude_target);
    

    // pose debug
    geometry_msgs::PoseStamped posemsg;
    posemsg.header.frame_id = "map";
    posemsg.pose.position.z = 1;
    posemsg.pose.orientation.x = quat_from_rpy.x();
    posemsg.pose.orientation.y = quat_from_rpy.y();
    posemsg.pose.orientation.z = quat_from_rpy.z();
    posemsg.pose.orientation.w = quat_from_rpy.w();
    // pub_direction_.publish(posemsg);


    publish_count++;
    if(publish_count > 10)
    {
        // std::cout<<"=======[controller_node.h]============="<<std::endl;
        // std::cout<<"command"<<std::endl;
        // std::cout<<
        // "roll : "<<euler_angles(0) * 180 / M_PI<<std::endl<<
        // "pitch : "<<euler_angles(1) * 180 / M_PI<<std::endl<<
        // "yaw : "<<euler_angles(2) * 180 / M_PI<<std::endl;


        //std::cout<<"==========================================="<<std::endl;
        // std::cout<<"thrust   : "<<(*ref_attitude_thrust)[3]<<std::endl
        //          <<"acc   : "<<(*ref_attitude_thrust)[3] / 1.5 <<std::endl
        //          <<"throttle : "<<throttle<<std::endl;
        publish_count=0;
        
    }

    // DEBUG for simulation
    /*
    mav_msgs::RollPitchYawrateThrust temp_command;
    temp_command.thrust.x = 0;
    temp_command.thrust.y = 0;
    temp_command.thrust.z = std::max(0.0, (*ref_attitude_thrust)[3]);

    temp_command.roll = (*ref_attitude_thrust)[0];
    temp_command.pitch = (*ref_attitude_thrust)[1];
    temp_command.yaw_rate = (*ref_attitude_thrust)[2];
    temp_command.header.stamp = ros::Time::now();  // TODO(acmarkus): get from msg
    pub_debug_.publish(temp_command);
    */


    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    marker.pose.orientation.w =1.0;


    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    
    marker.color.g = 1.0f;
    marker.color.a = 1.0;
    marker.type = visualization_msgs::Marker::POINTS;


    // marker.points.
    controller_->getPredictedState(marker);
    pub_marker_.publish(marker);
    
    // pub_debug_.publish(temp_command);

    // std::cout<<"publish command!"<<std::endl;
    // std::cout<<"orientation.x : "<<attitude_target.orientation.x<<"\n"
    //          <<"orientation.y : "<<attitude_target.orientation.x<<"\n"
    //          <<"orientation.z : "<<attitude_target.orientation.x<<"\n"
    //          <<"orientation.w : "<<attitude_target.orientation.x<<"\n"
    //          <<"thrust        : "<<attitude_target.thrust<<"\n";

}
#endif
