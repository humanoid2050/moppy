#include "ros/ros.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/Odometry.h"
#include "hardware_abstraction/platform_control.h"
#include "hardware_abstraction/switch_set.h"
#include "sensor_msgs/BatteryState.h"

#include "moppy/moppy.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "moppy");

    ros::NodeHandle n("~");

    std::string device;
    int baud;
    n.param("serial_device", device, std::string("/dev/ttyACM0"));
    n.param("baud", baud, 115200);

    double radius;
    double ticks_per_rev;
    double wheel_diameter;
    n.param("characteristic_radius", radius, 0.045);
    n.param("ticks_per_revolution", ticks_per_rev, 900.d);
    n.param("wheel_diameter", wheel_diameter, 0.04);


    Moppy moppy(device,baud);
    moppy.setHardwareParams(radius, ticks_per_rev, wheel_diameter);

    double P, I, D;
    n.param("left/P", P, 1.0);
    n.param("left/I", I, 1.0);
    n.param("left/D", D, 1.0);
    moppy.setPIDCoefficients(Moppy::MOTOR::LEFT,P,I,D);

    n.param("right/P", P, 1.0);
    n.param("right/I", I, 1.0);
    n.param("right/D", D, 1.0);
    moppy.setPIDCoefficients(Moppy::MOTOR::RIGHT,P,I,D);

    n.param("lidar/P", P, 1.0);
    n.param("lidar/I", I, 1.0);
    n.param("lidar/D", D, 1.0);
    moppy.setPIDCoefficients(Moppy::MOTOR::LIDAR,P,I,D);

    ros::Subscriber sub = n.subscribe("control", 1, &Moppy::onPlatformControl, dynamic_cast<DifferentialDrive*>(&moppy));

    ros::Publisher battery_pub = n.advertise<sensor_msgs::BatteryState>("battery", 10);

    ros::Publisher bumper_pub = n.advertise<hardware_abstraction::switch_set>("bumper", 10);

    ros::Publisher edge_pub = n.advertise<hardware_abstraction::switch_set>("edge", 10);

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf2_ros::TransformBroadcaster odom_br;

    moppy.setBatteryPublisher(
        [&battery_pub](const float& volts){
            sensor_msgs::BatteryState batt_msg;
            batt_msg.header.stamp = ros::Time::now();
            batt_msg.header.frame_id = "moppy";
            batt_msg.voltage = volts;
            batt_msg.current = NAN;
            batt_msg.charge = NAN;
            batt_msg.capacity = NAN;
            batt_msg.design_capacity = NAN;
            batt_msg.percentage = NAN;
            batt_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
            batt_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
            batt_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH;
            batt_msg.present = true;
            battery_pub.publish(batt_msg);
        }
    );

    moppy.setBumperPublisher(
        [&bumper_pub, &odom_br](const std::vector<uint8_t>& switches){
            hardware_abstraction::switch_set bumper_msg;
            bumper_msg.header.stamp = ros::Time::now();
            bumper_msg.header.frame_id = "moppy";
            bumper_msg.switch_state = switches;
            bumper_pub.publish(bumper_msg);
        }
    );

    moppy.setEdgePublisher(
        [&edge_pub](const std::vector<uint8_t>& switches){
            hardware_abstraction::switch_set edge_msg;
            edge_msg.header.stamp = ros::Time::now();
            edge_msg.header.frame_id = "moppy";
            edge_msg.switch_state = switches;
            edge_pub.publish(edge_msg);
        }
    );


    moppy.setOdomHandler(
        [&odom_pub,&odom_br](const Pose& pose, const Twist& twist){

            auto current_time = ros::Time::now();

            tf2::Quaternion q;
            q.setRPY(0, 0, pose.theta);

            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "moppy_odom";
            odom_trans.child_frame_id = "map";

            odom_trans.transform.translation.x = pose.x;
            odom_trans.transform.translation.y = pose.y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = tf2::toMsg(q);

            //send the transform
            odom_br.sendTransform(odom_trans);

            nav_msgs::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "moppy_odom";

            //set the position
            odom.pose.pose.position.x = pose.x;
            odom.pose.pose.position.y = pose.y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = tf2::toMsg(q);

            //set the velocity
            odom.child_frame_id = "map";
            odom.twist.twist.linear.x = twist.vx;
            odom.twist.twist.linear.y = twist.vy;
            odom.twist.twist.angular.z = twist.vtheta;

            //publish the message
            odom_pub.publish(odom);

        }
    );

    ros::spin();


}
