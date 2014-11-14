#include <ros/ros.h>
#include <functional>
#include <vector>

#include <s8_common_node/Node.h>
#include <s8_pose/pose_node.h>
#include <s8_utils/math.h>
#include <geometry_msgs/PoseStamped.h>

#define HZ                  50

using namespace s8::pose_node;
using namespace s8::utils::math;

class Pose : public s8::Node {
    ros::Publisher pose_publisher;
    double x;
    double z;
    FrontFacing front_facing;

public:
    Pose() : x(0.0), z(0.0), front_facing(FrontFacing::NORTH) {
        init_params();
        print_params();
        pose_publisher = nh.advertise<geometry_msgs::PoseStamped>(TOPIC_POSE, 1);
    }

    void update() {
        publish();
    }

    void publish() {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time();
        pose.pose.position.x = x;
        pose.pose.position.y = 0;
        pose.pose.position.z = z;

        switch(front_facing) {
            case FrontFacing::NORTH: pose.pose.orientation = degrees_to_quaternion(0); break;
            case FrontFacing::EAST: pose.pose.orientation = degrees_to_quaternion(90); break;
            case FrontFacing::SOUTH: pose.pose.orientation = degrees_to_quaternion(180); break;
            case FrontFacing::WEST: pose.pose.orientation = degrees_to_quaternion(270); break;
        }

        pose_publisher.publish(pose);
    }

private:
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
