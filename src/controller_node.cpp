#include <controller_node.h>


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_param("~");
  ControllerNode controller_node(nh, nh_param);
  ros::Rate loop_rate(30);

  // ros::Subscriber odom_subscriber = n.subscribe("/mavros/local_position/odom", 100, odomCallback);
  // // ros::Publisher  command_publisher = n.advertise<mavros_msgs::AttitudeTarget>("chatter", 1000);
  //mympc::ModelPredictiveController mpc_controller;

  while(ros::ok())
  {
    // mpc_controller.calculateRollPitchYawrateThrustCommand(ref_attitude_thrust);
    // controller_node.PublishCommand();
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}
