#include <ros/ros.h>
#include <functional>
#include <vector>

#include <s8_common_node/Node.h>
#include <s8_motor_controller/motor_controller_node.h>
#include <s8_pose/pose_node.h>
#include <s8_utils/math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <ras_arduino_msgs/Encoders.h>
#include <s8_pose/setPosition.h>
#include <s8_pose/setOrientation.h>

#define HZ                  10

#define TOPIC_ACTUAL_TWIST          s8::motor_controller_node::TOPIC_ACTUAL_TWIST
#define TOPIC_ENCODERS              s8::motor_controller_node::TOPIC_ENCODERS

#define PARAM_NAME_WHEEL_RADIUS                     "wheel_radius"
#define PARAM_NAME_ROBOT_BASE                       "robot_base"

#define PARAM_DEFAULT_WHEEL_RADIUS                  0.05
#define PARAM_DEFAULT_ROBOT_BASE                    0.215

#define TOPO_EAST           7 * M_PI / 4
#define TOPO_NORTH          1 * M_PI / 4
#define TOPO_WEST           3 * M_PI / 4
#define TOPO_SOUTH          5 * M_PI / 4

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
    double encoder_left, encoder_right;
    double tick_dist;
    bool isEncoderInitialized;

    FrontFacing front_facing;

    ros::ServiceServer set_orientation_service;
    ros::ServiceServer set_position_service;

public:
    Pose() : x(0.0), y(0.0), front_facing(FrontFacing::EAST) {
        init_params();
        print_params();
        pose_publisher = nh.advertise<geometry_msgs::PoseStamped>(TOPIC_POSE, 1);
        pose_simple_publisher = nh.advertise<geometry_msgs::Pose2D>(TOPIC_POSE_SIMPLE, 1);
        actual_twist_subscriber = nh.subscribe<geometry_msgs::Twist>(TOPIC_ACTUAL_TWIST, 1, &Pose::actual_twist_callback, this);
        encoder_subscriber = nh.subscribe<ras_arduino_msgs::Encoders>(TOPIC_ENCODERS, 1, &Pose::encoders_callback, this);

        velocity = 0.0;
        x = 0.0;
        y = 0.0;
        encoder_left = encoder_right = 0.0;
        theta = degrees_to_radians((double)front_facing_to_degrees(front_facing));
        tick_dist = 2*wheel_radius*M_PI/360;
        isEncoderInitialized = false;
        ROS_INFO("Robot is facing %s", to_string(front_facing).c_str());

        set_orientation_service = nh.advertiseService(SERVICE_SET_ORIENTATION, &Pose::set_orientation_callback, this);
        set_position_service = nh.advertiseService(SERVICE_SET_POSITION, &Pose::set_position_callback, this);
    }

    void update() {
        publish_rviz_pose();
        //publish_pose();
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
            //ROS_INFO("Linear velocity. %lf Sending pose.", velocity);

            double distance = velocity / HZ;
/*
            switch(front_facing) {
                case FrontFacing::EAST: x += distance; break;
                case FrontFacing::NORTH: y += distance; break;
                case FrontFacing::WEST: x -= distance; break;
                case FrontFacing::SOUTH: y -= distance; break;
            }

            publish_pose();
*/
        }
    }

    void encoders_callback(const ras_arduino_msgs::Encoders::ConstPtr & encoders) {

        if (isEncoderInitialized == false)
        {
            isEncoderInitialized = true;
            encoder_left = encoders->encoder2;
            encoder_right = encoders->encoder1;
            return;
        }
        /*
        if(velocity == 0)
        {
            isEncoderInitialized = false;
            ROS_INFO("initialized false");
            return;
        }
        */
        //ROS_INFO("encoders x: %lf, y: %lf", x, y);

        int current_left = encoders->encoder2;
        int current_right = encoders->encoder1;


        int delta_left = -delta(encoder_left, current_left);
        int delta_right = -delta(encoder_right, current_right);



        //if (delta_left > 50 || delta_right > 50)
        //    return;

        encoder_left = current_left;
        encoder_right = current_right;

        double delta_wheel_diff = delta_right*tick_dist-delta_left*tick_dist;
        double delta_theta = delta_wheel_diff/robot_base;

        double delta_wheel_sum = delta_right*tick_dist+delta_left*tick_dist;
        //ROS_INFO("delta_left: %d, delta_right: %d", delta_left, delta_right);
        double delta_dist  = delta_wheel_sum/2;
        double delta_x = delta_dist * std::cos(theta + delta_theta/2);
        double delta_y = delta_dist * std::sin(theta + delta_theta/2);

        //ROS_INFO("delta_x: %lf, delta_y: %lf", delta_x, delta_y);

        x = x + delta_x;
        y = y + delta_y;
        theta = theta + delta_theta;

        publish_pose();
    }

    int delta(int previous, int current){
        int diff = previous - current;
        if (diff > 60000)
            diff = 65536 - diff;
        else if (diff < -60000)
            diff = -65536 - diff;
        return diff;
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

    double get_heading(double theta){
        double heading = sign(theta) *degrees_to_radians((std::abs((int)radians_to_degrees(theta))%(int)radians_to_degrees(2*M_PI) ));

        if(heading < 0)
            heading += 2*M_PI;

        if(heading <= TOPO_NORTH || heading > TOPO_EAST) {
            return 0;
        }
        if(heading <= TOPO_WEST && heading > TOPO_NORTH) {
            return M_PI/2;
        }
        if(heading <= TOPO_SOUTH && heading > TOPO_WEST) {
            return M_PI;
        }
        if(heading <= TOPO_EAST && heading > TOPO_SOUTH) {
            return 3*M_PI/2;
        }
    }

    bool set_position_callback(s8_pose::setPosition::Request& request, s8_pose::setPosition::Response& response) {
        ROS_INFO("Position robot at: (%lf, %lf)", request.x, request.y);
        x = request.x;
        y = request.y;
        return true;
    }

    bool set_orientation_callback(s8_pose::setOrientation::Request& request, s8_pose::setOrientation::Response& response) {
        ROS_INFO("Resetting orientation");
        theta = get_heading(theta);
        return true;
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
