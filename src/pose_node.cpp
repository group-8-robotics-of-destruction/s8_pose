#include <ros/ros.h>
#include <functional>
#include <vector>

#include <s8_common_node/Node.h>
#include <s8_motor_controller/motor_controller_node.h>
#include <s8_turner/turner_node.h>
#include <s8_pose/pose_node.h>
#include <s8_utils/math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <s8_turner/TurnActionResult.h>

#define HZ                  10

#define TOPIC_ACTUAL_TWIST          s8::motor_controller_node::TOPIC_ACTUAL_TWIST
#define TOPIC_TURN_ACTION_RESULT    s8::turner_node::ACTION_TURN + "/result"

using namespace s8::pose_node;
using namespace s8::utils::math;

class Pose : public s8::Node {
    ros::Subscriber actual_twist_subscriber;
    ros::Subscriber turn_action_result_subscriber;
    ros::Publisher pose_publisher;
    ros::Publisher pose_simple_publisher;
    double x;
    double y;
    FrontFacing front_facing;

public:
    Pose() : x(0.0), y(0.0), front_facing(FrontFacing::EAST) {
        init_params();
        print_params();
        pose_publisher = nh.advertise<geometry_msgs::PoseStamped>(TOPIC_POSE, 1);
        pose_simple_publisher = nh.advertise<geometry_msgs::Pose2D>(TOPIC_POSE_SIMPLE, 1);
        actual_twist_subscriber = nh.subscribe<geometry_msgs::Twist>(TOPIC_ACTUAL_TWIST, 1, &Pose::actual_twist_callback, this);
        turn_action_result_subscriber = nh.subscribe<s8_turner::TurnActionResult>(TOPIC_TURN_ACTION_RESULT, 100, &Pose::turn_action_result_callback, this);

        ROS_INFO("Robot is facing %s", to_string(front_facing).c_str());
    }

    void update() {
        publish_rviz_pose();
        publish_pose();
    }

    void publish_rviz_pose() {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0;

        int rotation_degrees = front_facing_to_degrees(front_facing);

        pose.pose.orientation = degrees_to_quaternion(rotation_degrees);

        pose_publisher.publish(pose);
    }

    void publish_pose() {
        int rotation_degrees = front_facing_to_degrees(front_facing);

        geometry_msgs::Pose2D pose_simple;
        pose_simple.x = x;
        pose_simple.y = y;
        pose_simple.theta = degrees_to_radians(rotation_degrees);

        pose_simple_publisher.publish(pose_simple);
    }

private:
    void actual_twist_callback(const geometry_msgs::Twist::ConstPtr & twist) {
        double v = twist->linear.x;

        if(!is_zero(v)) {
            //Moving forward. Then send pose.
            ROS_INFO("Linear velocity. %lf Sending pose.", v);

            double distance = v / HZ;

            switch(front_facing) {
                case FrontFacing::EAST: x += distance; break;
                case FrontFacing::NORTH: y += distance; break;
                case FrontFacing::WEST: x -= distance; break;
                case FrontFacing::SOUTH: y -= distance; break;
            }

            publish_pose();
        }
    }

    void turn_action_result_callback(const s8_turner::TurnActionResult::ConstPtr & turn_action_result) {
        int status = turn_action_result->status.status;
        int degrees_turnt = turn_action_result->result.degrees;

        if(status == 3) { //Success
            ROS_INFO("Robot has turnt %d degrees", degrees_turnt);
            if(degrees_turnt < 0) {
                front_facing = minus_90_degrees(front_facing);
            } else if(degrees_turnt > 0) {
                front_facing = plus_90_degrees(front_facing);
            }

            ROS_INFO("Robot is now facing %s", to_string(front_facing).c_str());
        } else {
            ROS_WARN("Warning. Turner didn't succeed. Now assuming the robot hasnt moved.");
        }
    }

    int front_facing_to_degrees(FrontFacing front_facing) {
        return (int)front_facing;
    }

    FrontFacing minus_90_degrees(FrontFacing front_facing) {
        switch(front_facing) {
            case FrontFacing::EAST: return FrontFacing::SOUTH;
            case FrontFacing::SOUTH: return FrontFacing::WEST;
            case FrontFacing::WEST: return FrontFacing::NORTH;
            case FrontFacing::NORTH: return FrontFacing::EAST;
        }
    }

    FrontFacing plus_90_degrees(FrontFacing front_facing) {
        switch(front_facing) {
            case FrontFacing::EAST: return FrontFacing::NORTH;
            case FrontFacing::NORTH: return FrontFacing::WEST;
            case FrontFacing::WEST: return FrontFacing::SOUTH;
            case FrontFacing::SOUTH: return FrontFacing::EAST;
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
