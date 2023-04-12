#include <math.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <signal.h>
#include <termios.h>

#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_datatypes.h>

#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_LEFT 0x44
#define KEYCODE_RIGHT 0x43
#define KEYCODE_SPACE 0x20

int kfd = 0;
struct termios cooked, raw;

class UnitreeTeleop
{
public:
    UnitreeTeleop();
    void keyLoop();
    void pubForce(double x, double y, double z, double ax, double ay, double az);

private:
    double x, y, z, ax, ay, az;
    std::string robot_name;
    ros::NodeHandle nh_;
    ros::ServiceClient set_model_state_client_;
    ros::ServiceClient get_model_state_client_;
};

UnitreeTeleop::UnitreeTeleop()
{
    set_model_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    get_model_state_client_ = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    
    ros::param::get("/robot_name", robot_name);
    robot_name = robot_name + "_gazebo";

    ros::Rate loop_rate(30);
}

void UnitreeTeleop::pubForce(double x, double y, double z, double ax, double ay, double az)
{
    gazebo_msgs::GetModelState get_model_state;
    get_model_state.request.model_name = robot_name;
    if (!get_model_state_client_.call(get_model_state))
    {
        ROS_ERROR("Failed to call service /gazebo/get_model_state");
        return;
    }

    tf::Quaternion q(
        get_model_state.response.pose.orientation.x,
        get_model_state.response.pose.orientation.y,
        get_model_state.response.pose.orientation.z,
        get_model_state.response.pose.orientation.w);

    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    tf::Quaternion q_stand;
    q_stand.setRPY(0, 0, yaw);

    gazebo_msgs::SetModelState set_model_stand;
    set_model_stand.request.model_state.model_name = robot_name;
    set_model_stand.request.model_state.pose = get_model_state.response.pose;
    set_model_stand.request.model_state.pose.orientation.x = q_stand.getX();
    set_model_stand.request.model_state.pose.orientation.y = q_stand.getY();
    set_model_stand.request.model_state.pose.orientation.z = q_stand.getZ();
    set_model_stand.request.model_state.pose.orientation.w = q_stand.getW();

    if (!set_model_state_client_.call(set_model_stand))
    {
        ROS_ERROR("Failed to call service /gazebo/set_model_state");
        return;
    }

    gazebo_msgs::SetModelState set_model_state;
    set_model_state.request.model_state.model_name = robot_name;
    set_model_state.request.model_state.reference_frame = "base";
    // set_model_state.request.model_state.pose = get_model_state.response.pose;
    set_model_state.request.model_state.twist.linear.x = x;
    set_model_state.request.model_state.twist.linear.y = 0;
    set_model_state.request.model_state.twist.angular.z = az;

    if (!set_model_state_client_.call(set_model_state))
    {
        ROS_ERROR("Failed to call service /gazebo/set_model_state");
        return;
    }
    ros::spinOnce();
}

void UnitreeTeleop::keyLoop()
{
    char c;
    bool dirty = false;
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use 'Up/Down/Left/Right/SpaceBar' ");
    for (;;)
    {
        // get the next event from the keyboard
        if (read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }
        ROS_DEBUG("value: 0x%02X\n", c);
        switch (c)
        {
        case KEYCODE_UP:
            x = 2.0;
            y = 0.0;
            z = 0.0;
            ax = 0.0;
            ay = 0.0;
            az = 0.0;
            dirty = true;
            break;
        case KEYCODE_DOWN:
            x = -2.0;
            y = 0.0;
            z = 0.0;
            ax = 0.0;
            ay = 0.0;
            az = 0.0;
            dirty = true;
            break;
        case KEYCODE_LEFT:
            x = 0.0;
            y = 0.0;
            z = 0.0;
            ax = 0.0;
            ay = 0.0;
            az = 3.0;
            dirty = true;
            break;
        case KEYCODE_RIGHT:
            x = 0.0;
            y = 0.0;
            z = 0.0;
            ax = 0.0;
            ay = 0.0;
            az = -3.0;
            dirty = true;
            break;
        case KEYCODE_SPACE:
            x = 0.0;
            y = 0.0;
            z = 0.0;
            ax = 0.0;
            ay = 0.0;
            az = 0.0;
            dirty = true;
            break;
        }
        pubForce(x, y, z, ax, ay, az);
    }
    return;
}

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_teleop");

    UnitreeTeleop teleop;

    signal(SIGINT, quit);

    teleop.keyLoop();

    return (0);
}

