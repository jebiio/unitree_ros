#include <math.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <signal.h>
#include <termios.h>

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf/transform_datatypes.h>

#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_LEFT 0x44
#define KEYCODE_RIGHT 0x43
#define KEYCODE_SPACE 0x20

int kfd = 0;
struct termios cooked, raw;

class unitreeTeleop
{
public:
    unitreeTeleop();
    void keyLoop();
    void pubForce(double x, double y, double z, double ax, double ay, double az);
    void states_cb(const gazebo_msgs::ModelStates::ConstPtr &msg);

private:
    double x, y, z, ax, ay, az;
    ros::NodeHandle n;
    ros::Publisher force_pub;
    geometry_msgs::Wrench Force;
    gazebo_msgs::ModelState model_state_pub;
    ros::Publisher move_publisher;
    ros::Subscriber state_sub;
};

unitreeTeleop::unitreeTeleop()
{
    state_sub = n.subscribe("/gazebo/model_states", 1000, &unitreeTeleop::states_cb, this);
    move_publisher = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
    std::string robot_name;
    ros::param::get("/robot_name", robot_name);
    std::cout << "robot_name: " << robot_name << std::endl;

    model_state_pub.model_name = robot_name + "_gazebo";
    ros::Rate loop_rate(30);
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
    unitreeTeleop teleop;
    signal(SIGINT, quit);
    teleop.keyLoop();
    return (0);
}

void unitreeTeleop::pubForce(double x, double y, double z, double ax, double ay, double az)
{
    
    model_state_pub.twist.linear.x = x; // 0.02: 2cm/sec
    model_state_pub.twist.linear.y = y;
    model_state_pub.twist.linear.z = z;

    model_state_pub.twist.angular.x = ax;
    model_state_pub.twist.angular.y = ay;
    model_state_pub.twist.angular.z = az;

    model_state_pub.reference_frame = "base";

    move_publisher.publish(model_state_pub);
    ros::spinOnce();
}

void unitreeTeleop::states_cb(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    tf::Quaternion q(msg->pose[2].orientation.x, msg->pose[2].orientation.y, msg->pose[2].orientation.z, msg->pose[2].orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    tf::Quaternion q_zero_roll_pitch = tf::createQuaternionFromRPY(0, 0, yaw);

    model_state_pub.pose.orientation.x = q_zero_roll_pitch.getX();
    model_state_pub.pose.orientation.y = q_zero_roll_pitch.getY();
    model_state_pub.pose.orientation.z = q_zero_roll_pitch.getZ();
    model_state_pub.pose.orientation.w = q_zero_roll_pitch.getW();
}

void unitreeTeleop::keyLoop()
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
            x = 1;
            y = 0.0;
            z = 0.0;
            ax = 0.0;
            ay = 0.0;
            az = 0.0;
            dirty = true;
            break;
        case KEYCODE_DOWN:
            x = -1;
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
            az = 1.0;
            dirty = true;
            break;
        case KEYCODE_RIGHT:
            x = 0.0;
            y = 0.0;
            z = 0.0;
            ax = 0.0;
            ay = 0.0;
            az = -1.0;
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
