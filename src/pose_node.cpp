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
#include <ras_arduino_msgs/Encoders.h>

#define HZ                  10

#define TOPIC_ACTUAL_TWIST          s8::motor_controller_node::TOPIC_ACTUAL_TWIST
#define TOPIC_TURN_ACTION_RESULT    s8::turner_node::ACTION_TURN + "/result"
#define TOPIC_ENCODERS              s8::motor_controller_node::TOPIC_ENCODERS

#define PARAM_NAME_WHEEL_RADIUS                     "wheel_radius"
#define PARAM_NAME_ROBOT_BASE                       "robot_base"

#define PARAM_DEFAULT_WHEEL_RADIUS                  0.05
#define PARAM_DEFAULT_ROBOT_BASE                    0.215

using namespace s8::pose_node;
using namespace s8::utils::math;

class Pose : public s8::Node {
    ros::Subscriber actual_twist_subscriber;
    ros::Subscriber turn_action_result_subscriber;
    ros::Subscriber encoder_subscriber;
    ros::Publisher pose_publisher;
    ros::Publisher pose_simple_publisher;

    double x, y, theta;
    double velocity;
    double wheel_radius, robot_base;
    double tick_dist;

    FrontFacing front_facing;

public:
    Pose() : x(0.0), y(0.0), front_facing(FrontFacing::EAST) {
        init_params();
        print_params();
        pose_publisher = nh.advertise<geometry_msgs::PoseStamped>(TOPIC_POSE, 1);
        pose_simple_publisher = nh.advertise<geometry_msgs::Pose2D>(TOPIC_POSE_SIMPLE, 1);
        actual_twist_subscriber = nh.subscribe<geometry_msgs::Twist>(TOPIC_ACTUAL_TWIST, 1, &Pose::actual_twist_callback, this);
        turn_action_result_subscriber = nh.subscribe<s8_turner::TurnActionResult>(TOPIC_TURN_ACTION_RESULT, 100, &Pose::turn_action_result_callback, this);
        encoder_subscriber = nh.subscribe<ras_arduino_msgs::Encoders>(TOPIC_ENCODERS, 1, &Pose::encoders_callback, this);

        velocity = 0.0;
        x = 0.0;
        y = 0.0;
        theta = degrees_to_radians((double)front_facing_to_degrees(front_facing));
        tick_dist = 2*wheel_radius*M_PI/360;

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

        pose.pose.orientation = radians_to_quaternion(theta);

        pose_publisher.publish(pose);
    }

    void publish_pose() {
        int rotation_degrees = front_facing_to_degrees(front_facing);

        geometry_msgs::Pose2D pose_simple;
        pose_simple.x = x;
        pose_simple.y = y;
        pose_simple.theta = theta;

        pose_simple_publisher.publish(pose_simple);
    }

private:
    void actual_twist_callback(const geometry_msgs::Twist::ConstPtr & twist) {
        velocity = twist->linear.x;

        if(!is_zero(velocity)) {
            //Moving forward. Then send pose.
            ROS_INFO("Linear velocity. %lf Sending pose.", velocity);

            double distance = velocity / HZ;

            switch(front_facing) {
                case FrontFacing::EAST: x += distance; break;
                case FrontFacing::NORTH: y += distance; break;
                case FrontFacing::WEST: x -= distance; break;
                case FrontFacing::SOUTH: y -= distance; break;
            }

            //publish_pose();
        }
    }

    void encoders_callback(const ras_arduino_msgs::Encoders::ConstPtr & encoders) {
        if(velocity == 0)
            return;

        int delta_left = encoders->delta_encoder2;
        int delta_right = encoders->delta_encoder1;

        double delta_wheel_diff = delta_right*tick_dist-delta_left*tick_dist;
        double delta_theta = delta_wheel_diff/robot_base;

        double delta_dist  = delta_wheel_diff/2;
        double delta_x = delta_dist * std::cos(theta + delta_theta/2);
        double delta_y = delta_dist * std::sin(theta + delta_theta/2);

        x = x + delta_x;
        y = y + delta_y;
        theta = theta + delta_theta;

        publish_pose();
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
            theta = degrees_to_radians((double)front_facing_to_degrees(front_facing));

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
        add_param(PARAM_NAME_ROBOT_BASE, robot_base, PARAM_DEFAULT_ROBOT_BASE);
        add_param(PARAM_NAME_WHEEL_RADIUS, wheel_radius, PARAM_DEFAULT_WHEEL_RADIUS);
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
