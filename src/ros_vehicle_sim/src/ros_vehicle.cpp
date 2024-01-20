#include "ros/ros.h"
#include "raptor_dbw_msgs/AcceleratorPedalCmd.h"
#include "raptor_dbw_msgs/SteeringCmd.h"
#include "raptor_dbw_msgs/MiscReport.h"
#include "raptor_dbw_msgs/GearReport.h"
#include "raptor_dbw_msgs/Gear.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TransformStamped.h"

#include <std_msgs/Int64.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "bike.h"

class DynamicBikeNode {
    private:
        double a = 1.4025;
        double b = 2.908-1.4025;
        double m = 2300;
        double J = 3071;
        double Cf = 107816;
        double Cr = 173478;
        double K = 15.88;
        DynamicBike dynamic_bike = DynamicBike(a,b,m,J,Cf,Cr,K);
        ros::Subscriber steer_cmd_sub;
        ros::Subscriber acc_cmd_sub;
        ros::Publisher odom_pub;
        ros::Publisher gear_report_pub;
        ros::Publisher misc_report_pub;
        tf2_ros::TransformBroadcaster tf2_broadcaster;
        ros::Rate loop_rate = 50;
        double ros_time;
        double sim_time;
        double steer_cmd;
        double speed_cmd;
    public:
        DynamicBikeNode(ros::NodeHandle *nh) {
            // Initialize parameters.
            a = 1.4025;
            b = 2.908-1.4025;
            m = 2300;
            J = 3071;
            Cf = 107816;
            Cr = 173478;
            K = 15.88;

            // Create subscribers.
            steer_cmd_sub = nh->subscribe("vehicle/steering_cmd", 10, 
                &DynamicBikeNode::steerCallback, this);
            acc_cmd_sub = nh->subscribe("vehicle/accelerator_pedal_cmd", 10, 
                &DynamicBikeNode::accCallback, this);

            // Create publishers.
            odom_pub = nh->advertise<nav_msgs::Odometry>("novatel/oem7/odom", 1);
            gear_report_pub = nh->advertise<raptor_dbw_msgs::GearReport>(
                "/vehicle/gear_report", 10
            );
            misc_report_pub = nh->advertise<raptor_dbw_msgs::MiscReport>(
                "/vehicle/misc_report", 10
            );

            // Initialize vehicle time.
            ros_time = ros::Time::now().toSec();
            dynamic_bike.SetTime(ros_time);
            sim_time = dynamic_bike.GetTime();

            // Initialize vehicle state.
            dynamic_bike.SetYaw(-45 * 3.14 / 180.0); // 3.14 / 2.0
        }

        void steerCallback(const raptor_dbw_msgs::SteeringCmd& msg) {
            steer_cmd = msg.angle_cmd;
        }

        void accCallback(const raptor_dbw_msgs::AcceleratorPedalCmd& msg) {
            speed_cmd = msg.speed_cmd;
        }

        void run() {
            // Loop until node is killed.
            while (ros::ok()) {
                ////////////////////
                // Get inputs.
                ////////////////////
                
                ros::spinOnce();
                double inputs[2] = {steer_cmd, speed_cmd};
                dynamic_bike.SetInputs(inputs);

                ////////////////////
                // Simulate.
                ////////////////////

                ros_time = ros::Time::now().toSec();
                sim_time = dynamic_bike.GetTime();
                while (sim_time < ros_time) {
                    dynamic_bike.Advance();
                    sim_time = dynamic_bike.GetTime();
                }
                
                ////////////////////
                // Publish topics.
                ////////////////////

                // Get vehicle position and velocity.
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, dynamic_bike.GetYaw());
                q.normalize();
                double pos[2];
                dynamic_bike.GetPos(pos);
                double pos_dt[2];
                dynamic_bike.GetPosDt(pos_dt);

                // Publish the transform over tf.
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = ros::Time(ros_time);
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base_link";

                odom_trans.transform.translation.x = pos[0];
                odom_trans.transform.translation.y = pos[1];
                odom_trans.transform.rotation = tf2::toMsg(q);

                tf2_broadcaster.sendTransform(odom_trans);

                // Publish odom message.
                nav_msgs::Odometry odom;
                odom.header.stamp = ros::Time(ros_time);
                odom.header.frame_id = "odom";
                odom.child_frame_id = "base_link";

                odom.twist.twist.linear.x = pos_dt[0];
                odom.twist.twist.linear.y = pos_dt[1];
                odom.twist.twist.angular.z = dynamic_bike.GetYawRate();
                
                odom.pose.pose.position.x = pos[0];
                odom.pose.pose.position.y = pos[1];
                odom.pose.pose.orientation = tf2::toMsg(q);

                odom_pub.publish(odom);

                // Publish misc report.
                raptor_dbw_msgs::MiscReport misc_report_msg;
                misc_report_msg.by_wire_ready = true;
                misc_report_msg.drive_by_wire_enabled = true;
                misc_report_msg.comms_fault = false;
                misc_report_msg.general_actuator_fault = false;
                misc_report_msg.general_driver_activity = false;

                misc_report_pub.publish(misc_report_msg);

                // Publish gear report.
                raptor_dbw_msgs::GearReport gear_report_msg;
                gear_report_msg.state.gear = raptor_dbw_msgs::Gear::DRIVE;

                gear_report_pub.publish(gear_report_msg);

                ////////////////////
                // Wait.
                ////////////////////

                loop_rate.sleep();
            }
        }
};


int main(int argc, char **argv) {

    ros::init(argc, argv, "vehicle_sim");
    ros::NodeHandle nh;
    DynamicBikeNode node = DynamicBikeNode(&nh);
    node.run();

    return 0;
}