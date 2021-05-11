//
// Created by admin-vras on 19.03.21.
//
/* This node should take a world -> ground_truth and map -> base_link transformations from simulator and calculates a
 * differential transform between the world -> map based on a presumption that base_link = ground_truth in t=0.
 *
 */
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <cstring>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>


/* this node creates differential Transformation from two transforms with same /frame. Illustration lower
 *  From transforms a->x1, b->x2, while (x1 = x2) creates a->b.
 *  a ----\     ->  a
 *          x   ->  |   x
 *  b-----/     ->  b
 *
*/
std::string trans1a = "world";
std::string trans1b = "X1_ground_truth";
std::string trans2a = "X1/map";
std::string trans2b = "X1";
bool transCreated = false;

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_tree_connect");
    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped tfStaticTransform;
    geometry_msgs::TransformStamped worldTrans;
    geometry_msgs::TransformStamped mapTrans;
    ros::NodeHandle node;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ROS_INFO("class initialised");
    node.param<std::string>("tree_1_From", trans1a,"world");
    node.param<std::string>("tree_1_To", trans1b, "X1_ground_truth");
    node.param<std::string>("tree_2_From", trans2a, "X1/map");
    node.param<std::string>("tree_2_To", trans2b, "X1");

    ros::Duration(1).sleep();
    ROS_INFO("sleep done");
    if(!transCreated){
        // all necessary transforms are available
        //ROS_INFO("We want transform %s -> %s\n %s -> %s",this->trans1a.c_str(),this->trans1b.c_str(),this->trans2a.c_str(),this->trans2b.c_str());
        //ROS_INFO("%d",this->tfBuffer.canTransform(this->trans1b,this->trans1a,ros::Time(0)));
        //ROS_INFO("%d",this->tfBuffer.canTransform(this->trans2b,this->trans2a,ros::Time(0)));
        if(true){
            // calculate the differetial transform JUST ONCE!
            try {
                worldTrans = tfBuffer
                        .lookupTransform(trans1b,
                                         trans1a,
                                         ros::Time(0));
            } catch (tf2::TransformException &ex){
                ROS_WARN("Could NOT transform world to ground_truth: %s", ex.what());
            }

            try {
                mapTrans = tfBuffer
                        .lookupTransform(trans2b,
                                         trans2a,
                                         ros::Time(0));
            } catch (tf2::TransformException &ex) {
                ROS_WARN("Could NOT transform X1/map to X1: %s", ex.what());
            }
            Eigen::Isometry3d worldMapTrans = tf2::transformToEigen(mapTrans.transform)*tf2::transformToEigen(worldTrans.transform).inverse();
            geometry_msgs::TransformStamped static_world_map_transform = tf2::eigenToTransform(worldMapTrans);
            static_world_map_transform.header.stamp = ros::Time::now();
            static_world_map_transform.header.frame_id = trans1a;
            static_world_map_transform.child_frame_id = trans2a;
            static_world_map_transform.header.stamp = mapTrans.header.stamp;
            static_broadcaster.sendTransform(static_world_map_transform);
            transCreated = true;
        }
    }
    ROS_INFO("sub done");
    ros::spin();
    return 0;
}

