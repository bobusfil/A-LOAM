#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <cstring>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Float64.h>

double trajectoryLength;
bool tfDetected = false;
Eigen::Isometry3d currentTransform;
Eigen::Isometry3d newTransform;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void lengthIncrement(const tf2_msgs::TFMessage& msg)
{
    // if selected transform is detected
    for (const auto& transform : msg.transforms) {
        if (strcmp(transform.child_frame_id.c_str(), "X1") == 0 && strcmp(transform.header.frame_id.c_str(), "X1/map") == 0) {
            if(tfDetected == false){
                currentTransform = tf2::transformToEigen(transform);
                tfDetected = true;
            } else{
                // calculating the transform difference
                newTransform = tf2::transformToEigen(transform);
                Eigen::Isometry3d diffTransformEig = currentTransform*newTransform.inverse();
                geometry_msgs::TransformStamped diffTransform = tf2::eigenToTransform(diffTransformEig);
                double newIncrement = sqrt(
                            pow(diffTransform.transform.translation.x,2) +
                            pow(diffTransform.transform.translation.y,2) +
                            pow(diffTransform.transform.translation.z,2));
                currentTransform = newTransform;
                //ROS_INFO("increment: %.4lf",newIncrement);
                if (0.001 < newIncrement && newIncrement < 0.5){
                    trajectoryLength = trajectoryLength + newIncrement;
            }
                break;
            }
        }
    }

}

int main(int argc, char **argv)
{
    trajectoryLength = 0;
    ros::init(argc, argv, "pathLength");
    ros::NodeHandle n;
    ros::Rate rate(1);
    ros::Subscriber sub = n.subscribe("tf", 1000, lengthIncrement);
    while(ros::ok()) {
        ros::spinOnce();
        ROS_INFO("Trajectory length: %.4lf",trajectoryLength);
        rate.sleep();
    }
}