#include <cstdlib>

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>

void call_mode(ros::ServiceClient cl, mavros_msgs::SetMode set_mode)
{
    if (cl.call(set_mode)) {
        ROS_INFO("setmode send ok value");
    } else {
        ROS_ERROR("Failed SetMode");
    }
}

void set_arm_status(ros::ServiceClient cl, mavros_msgs::CommandBool cmd)
{
    if (cl.call(cmd)) {
        ROS_INFO("ARM send ok %d", cmd.response.success);
    } else {
        ROS_ERROR("Failed arming or disarming");
    }
}

// NOTE: Need to call this with argument of what mode
int main(int argc, char **argv)
{

    ros::init(argc, argv, "mavros_test_setmode");
    ros::NodeHandle n;

    ros::Rate r(20);  // need to send set_point commands at least 20Hz

    // Set Mode Client
    ros::ServiceClient set_mode_cl = (
        n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode"));

    // Arming Client
    ros::ServiceClient arming_cl = (
        n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming"));

    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.custom_mode = argv[1];

    // Set arming status to True
    mavros_msgs::CommandBool srv_arm;
    srv_arm.request.value = true;

    // Set pixhawk mode to "AUTO.LAND"
    call_mode(set_mode_cl, srv_setMode);

    // Arm drone
    // set_arm_status(arming_cl, srv_arm);

    return 0;
}
