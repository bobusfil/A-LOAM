#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <cstring>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>


/* this node creates differential Transformation from two transforms with same /frame. Illustration lower
 *  From transforms a->x1, b->x2, while (x1 = x2) creates a->b.
 *  a ----\     ->  a
 *          x   ->  |   x
 *  b-----/     ->  b
 *
*/

// set default params
std::string frameA;
std::string frameB;
std::string frameX_1;
std::string frameX_2;


//declaration of tfStaticTransform as global variable so the callback fcn
// will have access to it.
geometry_msgs::TransformStamped tfStaticTransform;

/* This fcn will run each time new transform is published,
 * it will check whether the published transform is the one
 * we want and then runs calculations for diff. transform.
 * */
void diffTransformCalc(const tf2_msgs::TFMessage& msg){
    // debugging print
    //ROS_INFO(msg.transforms.data()->child_frame_id.c_str());
    // run only if child frame_id is the desired one (/aft_mapped in our case)

    // initialising the variables for transform calculations
    tf2_msgs::TFMessage mapToLaser = msg;
    geometry_msgs::TransformStamped LaserToRobot = tfStaticTransform;
    Eigen::Isometry3d diffTrans;


    std::string frameX_1_w_slash = "/" + frameX_1;
    // if selected transform is detected
    if(strcmp(msg.transforms.data()->child_frame_id.c_str(),frameX_1_w_slash.c_str()) == 0){

        // declaring a tf Broadcaster of the differencial Transform
        static tf2_ros::TransformBroadcaster br;

        // converting the tf2_msgs::TFMessages to Eigen::Isometry3d messages which are capable of the multiplication.
        Eigen::Isometry3d l2r = tf2::transformToEigen(LaserToRobot);
        Eigen::Isometry3d m2l = tf2::transformToEigen(mapToLaser.transforms.data()->transform);

        // calculating the differential transform from a->b and c->b to a->c
        diffTrans = m2l*l2r.inverse();

        // converting the transform format back to tgeometry_msgs:TransformStamped
        geometry_msgs::TransformStamped diffTfTransform = tf2::eigenToTransform(diffTrans);

        diffTfTransform.header.frame_id = frameA;
        diffTfTransform.child_frame_id = frameB;

        br.sendTransform(diffTfTransform);
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "diffTransNode");
    ros::NodeHandle node;

    // load params from launchfile
    node.param<std::string>("frame1From",frameA,"camera_init");
    node.param<std::string>("frame1To",frameX_1,"aft_mapped");
    node.param<std::string>("frame2From",frameB,"X1");
    node.param<std::string>("frame2To",frameX_2,"X1/base_link/front_laser");


    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer, node, true);

    ros::Subscriber sub = node.subscribe("tf", 10, diffTransformCalc);
    tfStaticTransform = tfBuffer.lookupTransform(frameB, frameX_2, ros::Time(0),ros::Duration(10));

    // Setting the node rate
    ros::spin();

    return 0;
}