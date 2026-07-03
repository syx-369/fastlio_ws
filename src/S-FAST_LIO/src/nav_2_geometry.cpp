#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// 全局发布者变量
ros::Publisher fastpose_pub;
ros::Publisher fasttwist_pub;
ros::Publisher localizer_pose_pub;
// 回调函数：处理接收到的nav_msgs::Odometry消息
void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    // 提取位姿信息并发布
    geometry_msgs::PoseStamped poseStamped;
    geometry_msgs::PoseStamped localizer_pose_msg;
    poseStamped.header = odom->header;
    poseStamped.header.frame_id = "map";
    poseStamped.pose = odom->pose.pose;

    localizer_pose_msg = poseStamped;
    localizer_pose_msg.pose.position.z += 0.25;
    fastpose_pub.publish(poseStamped);
    localizer_pose_pub.publish(localizer_pose_msg);

    // 打印位姿信息
    ROS_INFO("PoseStamped:");
    ROS_INFO("Position - x: [%f], y: [%f], z: [%f]", poseStamped.pose.position.x, poseStamped.pose.position.y, poseStamped.pose.position.z);
    ROS_INFO("Orientation - w: [%f], x: [%f], y: [%f], z: [%f]", poseStamped.pose.orientation.w, poseStamped.pose.orientation.x, poseStamped.pose.orientation.y, poseStamped.pose.orientation.z);

    // 提取速度信息并发布
    geometry_msgs::TwistStamped twistStamped;
    twistStamped.header = odom->header;
    twistStamped.twist = odom->twist.twist;
    // if(twistStamped.twist.linear.x <= 0){twistStamped.twist.linear.x = - twistStamped.twist.linear.x;}
    fasttwist_pub.publish(twistStamped);

    // 打印速度信息
    ROS_INFO("TwistStamped:");
    ROS_INFO("Linear Velocity - x: [%f], y: [%f], z: [%f]", twistStamped.twist.linear.x, twistStamped.twist.linear.y, twistStamped.twist.linear.z);
    ROS_INFO("Angular Velocity - x: [%f], y: [%f], z: [%f]", twistStamped.twist.angular.x, twistStamped.twist.angular.y, twistStamped.twist.angular.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_to_pose_twist");
    ros::NodeHandle nh;

    // 创建发布者，发布到/pose_stamped和/twist_stamped话题
    fastpose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_stamped", 10000);
    fasttwist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist_stamped", 10000);
    localizer_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/localizer_pose", 10000);
    // 订阅/odom话题，回调函数处理接收到的消息
    ros::Subscriber fastsub = nh.subscribe("Odometry", 10000, odometryCallback);

    // 循环等待回调函数
    ros::spin();

    return 0;
}
