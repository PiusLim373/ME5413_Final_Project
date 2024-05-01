#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_utils/GetPoseFromPixel.h>
#include <pcl_conversions/pcl_conversions.h>

pcl::PointCloud<pcl::PointXYZ> camera_pcl;

void chatterCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, camera_pcl);
}

bool add(pcl_utils::GetPoseFromPixel::Request &req,
         pcl_utils::GetPoseFromPixel::Response &res)
{
    geometry_msgs::PoseStamped coor_wrt_camera;
    ROS_INFO("[Pointcloud Transformer] Querying point cloud at pixel coor: (%d, %d)", req.u, req.v);
    pcl::PointXYZ point;
    try
    {
        point = camera_pcl.at(req.u, req.v);
    }
    catch (pcl::IsNotDenseException &err)
    {
        ROS_ERROR("[Pointcloud Transformer] cloud not valid, have you get depth data before?");
        res.success = false;
        return true;
    }
    ROS_INFO("[Pointcloud Transformer] x: [%f]; y: [%f]; z: [%f]", point.x, point.y, point.z);
    if (isnan(point.x) || isnan(point.y) || isnan(point.z))
    {
        ROS_INFO("[Pointcloud Transformer] couldn't determine depth, probably too far");
        res.success = false;
        return true;
    }
    else
    {
        coor_wrt_camera.header.stamp = ros::Time::now();
        coor_wrt_camera.header.frame_id = "camera_front_depth_optical_frame";
        coor_wrt_camera.pose.position.x = point.x;
        coor_wrt_camera.pose.position.y = point.y;
        coor_wrt_camera.pose.position.z = point.z - 0.5;
        coor_wrt_camera.pose.orientation.x = 0;
        coor_wrt_camera.pose.orientation.y = 0;
        coor_wrt_camera.pose.orientation.z = 0;
        coor_wrt_camera.pose.orientation.w = 1;

        res.coor_wrt_camera = coor_wrt_camera;
        res.success = true;
        return true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_transformer");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/camera_front/depth/points", 1000, chatterCallback);
    ros::ServiceServer service = n.advertiseService("get_camera_pose_from_pixel", add);
    ros::spin();
    return 0;
}