#include "../../include/plane_detector.h"

void Pdetect::feet_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr &recived_pcl_msg)
{

    pcl::fromROSMsg(*recived_pcl_msg, footcloud);
    // cout<<footcloud.size()<<"-----------------"<<endl;

}