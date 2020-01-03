#include "../../include/robotodom2.h"
#include "tf_conversions/tf_eigen.h"

void Robotodom2::pclCallback(const sensor_msgs::PointCloud2::ConstPtr &recived_pcl_msg)
{
    sensor_msgs::PointCloud2 robot_pcl2;
    pcl_ros::transformPointCloud("t265_odom_frame", *recived_pcl_msg, robot_pcl2, listener);
    robot_pcl2.header.frame_id = "t265_odom_frame";
    pub.publish(robot_pcl2);

    pcl::PointCloud<pcl::PointXYZ> feet_cloud;
    if (fl_init == true && fr_init == true && bl_init == true && br_init == true)
    {
        feet_cloud.push_back(fl);
        feet_cloud.push_back(fr);
        feet_cloud.push_back(bl);
        feet_cloud.push_back(br);

        sensor_msgs::PointCloud2 output;
        pcl::PCLPointCloud2 cloud_p;
        pcl::toPCLPointCloud2(feet_cloud, cloud_p);
        pcl_conversions::fromPCL(cloud_p, output);
        output.header.frame_id = "t265_odom_frame";
        pub2.publish(output);
    }
}

void Robotodom2::flCallback(const sensor_msgs::PointCloud2::ConstPtr &recived_pcl_msg)
{
    sensor_msgs::PointCloud2 robot_pcl2;
    pcl_ros::transformPointCloud("t265_odom_frame", *recived_pcl_msg, robot_pcl2, listener2);
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(robot_pcl2, *cloud);

    if (cloud->size() == 1)
    {
        fl_init = true;
        fl = cloud->points[0];
    }
}

void Robotodom2::frCallback(const sensor_msgs::PointCloud2::ConstPtr &recived_pcl_msg)
{
    sensor_msgs::PointCloud2 robot_pcl2;
    pcl_ros::transformPointCloud("t265_odom_frame", *recived_pcl_msg, robot_pcl2, listener3);
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(robot_pcl2, *cloud);

    if (cloud->size() == 1)
    {
        fr_init = true;
        fr = cloud->points[0];
    }
}

void Robotodom2::blCallback(const sensor_msgs::PointCloud2::ConstPtr &recived_pcl_msg)
{
    sensor_msgs::PointCloud2 robot_pcl2;
    pcl_ros::transformPointCloud("t265_odom_frame", *recived_pcl_msg, robot_pcl2, listener4);
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(robot_pcl2, *cloud);

    if (cloud->size() == 1)
    {
        bl_init = true;
        bl = cloud->points[0];
    }
}

void Robotodom2::brCallback(const sensor_msgs::PointCloud2::ConstPtr &recived_pcl_msg)
{
    sensor_msgs::PointCloud2 robot_pcl2;
    pcl_ros::transformPointCloud("t265_odom_frame", *recived_pcl_msg, robot_pcl2, listener5);
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(robot_pcl2, *cloud);

    if (cloud->size() == 1)
    {
        br_init = true;
        br = cloud->points[0];
    }
}