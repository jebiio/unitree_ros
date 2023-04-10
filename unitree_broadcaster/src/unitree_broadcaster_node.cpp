#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "gazebo_msgs/ModelStates.h"

geometry_msgs::TransformStamped transformStamped;

void poseCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    transformStamped.transform.translation.x = msg->pose[2].position.x;
    transformStamped.transform.translation.y = msg->pose[2].position.y;
    transformStamped.transform.translation.z = msg->pose[2].position.z;
    transformStamped.transform.rotation.x = msg->pose[2].orientation.x;
    transformStamped.transform.rotation.y = msg->pose[2].orientation.y;
    transformStamped.transform.rotation.z = msg->pose[2].orientation.z;
    transformStamped.transform.rotation.w = msg->pose[2].orientation.w;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "unitree_broadcaster_node");

    ros::NodeHandle private_node("~");

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/gazebo/model_states", 10, &poseCallback);

    ros::Rate loop_rate(50);
    static tf2_ros::TransformBroadcaster odom_to_basefootprint, basefootprint_to_base;

    while (ros::ok())
    {
        geometry_msgs::TransformStamped ts_bfp, ts_base;
        ts_bfp = transformStamped;
        ts_bfp.transform.translation.z = 0.0;

        ts_bfp.header.frame_id = "odom";
        ts_bfp.child_frame_id = "base_footprint";
        ts_bfp.header.stamp = ros::Time::now();
        odom_to_basefootprint.sendTransform(ts_bfp);

        ts_base.transform.translation.x = 0.0;
        ts_base.transform.translation.y = 0.0;
        ts_base.transform.translation.z = transformStamped.transform.translation.z;
        ts_base.transform.rotation.x = 0.0;
        ts_base.transform.rotation.y = 0.0;
        ts_base.transform.rotation.z = 0.0;
        ts_base.transform.rotation.w = 1.0;
        ts_base.header.frame_id = "base_footprint";
        ts_base.child_frame_id = "base";
        ts_base.header.stamp = ros::Time::now();
        basefootprint_to_base.sendTransform(ts_base);

        loop_rate.sleep();
        ros::spinOnce();
    }


        ros::spin();
        return 0;
    };