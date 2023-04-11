#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_datatypes.h>

ros::ServiceClient set_state_client;
ros::ServiceClient get_state_client;
std::string robot_name;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    gazebo_msgs::GetModelState get_state_srv;
    get_state_srv.request.model_name = robot_name;
    if (!get_state_client.call(get_state_srv))
    {
        ROS_ERROR("Failed to call service /gazebo/get_model_state");
        return;
    }

    tf::Quaternion q(
        get_state_srv.response.pose.orientation.x,
        get_state_srv.response.pose.orientation.y,
        get_state_srv.response.pose.orientation.z,
        get_state_srv.response.pose.orientation.w);

    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    tf::Quaternion q_stand;
    q_stand.setRPY(0, 0, yaw);

    gazebo_msgs::SetModelState set_model_stand;
    set_model_stand.request.model_state.model_name = robot_name;
    set_model_stand.request.model_state.pose = get_state_srv.response.pose;
    set_model_stand.request.model_state.pose.orientation.x = q_stand.getX();
    set_model_stand.request.model_state.pose.orientation.y = q_stand.getY();
    set_model_stand.request.model_state.pose.orientation.z = q_stand.getZ();
    set_model_stand.request.model_state.pose.orientation.w = q_stand.getW();

    if (!set_state_client.call(set_model_stand))
    {
        ROS_ERROR("Failed to call service /gazebo/set_model_state");
        return;
    }

    gazebo_msgs::SetModelState set_model_state;
    set_model_state.request.model_state.model_name = "laikago_gazebo";
    set_model_state.request.model_state.reference_frame = "base";
    // set_model_state.request.model_state.pose = get_model_state.response.pose;
    set_model_state.request.model_state.twist.linear.x = msg->linear.x;
    set_model_state.request.model_state.twist.linear.y = msg->linear.y;
    set_model_state.request.model_state.twist.angular.z = msg->angular.z;

    if (!set_state_client.call(set_model_state))
    {
        ROS_ERROR("Failed to call service /gazebo/set_model_state");
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "unitree_base_controll");
    ros::NodeHandle n;

    ros::param::get("/robot_name", robot_name);
    robot_name = robot_name + "_gazebo";

    set_state_client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    get_state_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    ros::Subscriber sub = n.subscribe("/robot/unitree_base_control/cmd_vel", 1000, cmdVelCallback);

    ros::spin();

    return 0;
}