#include <ros/ros.h>
#include <functional>
#include <vector>

#include <s8_common_node/Node.h>
#include <s8_pose/pose_node.h>
#include <s8_utils/math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>

#define HZ                  10

using namespace s8::pose_node;
using namespace s8::utils::math;

class Pose : public s8::Node {
    ros::Publisher pose_publisher;
    ros::Publisher pose_simple_publisher;
    double x;
    double y;
    FrontFacing front_facing;

public:
    Pose() : x(0.0), y(0.0), front_facing(FrontFacing::NORTH) {
        init_params();
        print_params();
        pose_publisher = nh.advertise<geometry_msgs::PoseStamped>(TOPIC_POSE, 1);
        pose_simple_publisher = nh.advertise<geometry_msgs::Pose2D>(TOPIC_POSE_SIMPLE, 1);
    }

    void update() {
        publish();
    }

    void publish() {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0;

        int rotation_degrees = front_facing_to_degrees(front_facing);

        pose.pose.orientation = degrees_to_quaternion(rotation_degrees);

        pose_publisher.publish(pose);

        geometry_msgs::Pose2D pose_simple;
        pose_simple.x = x;
        pose_simple.y = y;
        pose_simple.theta = degrees_to_radians(rotation_degrees);

        pose_simple_publisher.publish(pose_simple);
    }

private:
    int front_facing_to_degrees(FrontFacing) {
        switch(front_facing) {
            case FrontFacing::NORTH: return 90;
            case FrontFacing::EAST: return 0;
            case FrontFacing::SOUTH: return 270;
            case FrontFacing::WEST: return 180;
        }
    }

    void init_params() {

    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);

    Pose pose;
    ros::Rate loop_rate(HZ);
    while(ros::ok()) {
        ros::spinOnce();
        pose.update();
        loop_rate.sleep();
    }

    return 0;
}
