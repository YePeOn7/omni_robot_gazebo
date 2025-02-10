#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

using namespace std;

class OmniOdometry
{
public:
    OmniOdometry(double tetha = 0.0) : tetha(tetha)
    {
        ros::NodeHandle nh;

        // Subscribers
        joint_state_sub = nh.subscribe("/robot_omni/joint_states", 10, &OmniOdometry::jointStateCallback, this);
        imu_sub = nh.subscribe("/robot_omni/imu/data", 10, &OmniOdometry::imuCallback, this);

        // joint_state_sub = nh.subscribe("/robot0/joint_states", 10, &OmniOdometry::jointStateCallback, this);
        // imu_sub = nh.subscribe("/robot0/imu/data", 10, &OmniOdometry::imuCallback, this);

        // Publisher
        odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

        // Load parameters
        nh.param("r", r, 0.028);
        nh.param("L", L, 0.088);

        double C = 2 * r / sqrt(3);
        double h = 0.0;
        mOd[0][0] = C * (cos(2 * M_PI / 3 + h) - (cos(2 * M_PI / 3 + h) - cos(h)) / 3);
        mOd[0][1] = C * (-cos(h) - (cos(2 * M_PI / 3 + h) - cos(h)) / 3);
        mOd[0][2] = C * (-(cos(2 * M_PI / 3 + h) - cos(h)) / 3);
        mOd[1][0] = C * (sin(2 * M_PI / 3 + h) - (sin(2 * M_PI / 3 + h) - sin(h)) / 3);
        mOd[1][1] = C * (-sin(h) - (sin(2 * M_PI / 3 + h) - sin(h)) / 3);
        mOd[1][2] = C * (-(sin(2 * M_PI / 3 + h) - sin(h)) / 3);

        x = 0.0;
        y = 0.0;
        yaw = 0.0;
        last_time = ros::Time::now();
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
    {
        using namespace std;
        if (msg->velocity.size() < 3)
        {
            ROS_WARN("Received joint state message with less than 3 velocities!");
            return;
        }

        // Extract wheel angular velocities
        double w1 = 0;
        double w2 = 0;
        double w3 = 0;

        // Loop through all joint names and extract the velocities for the specified joints
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == "first_wheel_joint")
            {
                w1 = msg->velocity[i];
            }
            else if (msg->name[i] == "second_wheel_joint")
            {
                w2 = msg->velocity[i];
            }
            else if (msg->name[i] == "third_wheel_joint")
            {
                w3 = msg->velocity[i];
            }
        }

        // Convert to linear velocity
        double V1 = r * w1;
        double V2 = r * w2;
        double V3 = r * w3;

        // Compute robot velocity using inverse kinematics
        // double Vx = (2.0 / 3.0) * (V1 - 0.5 * V2 - 0.5 * V3);
        // double Vy = (-2.0 / (3.0 * sqrt(3))) * (V2 - V3);
        // double Vx = (-2 * V1 * sin(tetha) + 2 * V2 * cos(tetha) + V3 / L) / 3;
        // double Vy = (2 * V1 * sin(tetha + 120) - 2 * V2 * cos(tetha + 120) + V3 / L) / 3;

        double Vx = (mOd[0][0] * w1 + mOd[0][1] * w2 + mOd[0][2] * w3);
        double Vy = (mOd[1][0] * w1 + mOd[1][1] * w2 + mOd[1][2] * w3);

        // Get current time
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        last_time = current_time;

        // Update position using IMU yaw
        x += Vx * cos(yaw) * dt - Vy * sin(yaw) * dt;
        y += Vx * sin(yaw) * dt + Vy * cos(yaw) * dt;

        // Publish odometry
        // ROS_INFO("vx: %.2f vy: %.2f  x: %2f, y: %2f, yaw: %2f", Vx, Vy, x, y, yaw);
        // ROS_INFO("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f", x, y, w1, w2, w3, Vx, Vy, yaw);
        publishOdometry(Vx, Vy, yaw, current_time);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        // Convert quaternion to yaw (heading)
        tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, yaw); // Extract yaw
    }

private:
    ros::Subscriber joint_state_sub, imu_sub;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster odom_broadcaster;

    double x, y, yaw;
    double r; // Wheel radius
    double L; // Base radius
    double tetha;
    double mOd[2][3] = {{0, 0, 0}, {0, 0, 0}};
    ros::Time last_time;

    void publishOdometry(double Vx, double Vy, double yaw, ros::Time current_time)
    {
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        // Position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        // Velocity
        odom.twist.twist.linear.x = Vx;
        odom.twist.twist.linear.y = Vy;
        odom.twist.twist.angular.z = yaw;

        // Publish odometry message
        odom_pub.publish(odom);

        // Publish transform
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(yaw);

        odom_broadcaster.sendTransform(odom_trans);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omni_odometry");
    OmniOdometry omni_odometry;
    ros::spin();
    return 0;
}
