#include "../../include/plane_detector.h"

Pdetect::Pdetect(){

    n.getParam("plane_detector/map_publish_topic",map_publish_topic);
    n.getParam("plane_detector/pcl_subscribe_topic",pcl_subscribe_topic);
    n.getParam("plane_detector/fieldname",fieldname);
    n.getParam("plane_detector/passlimit",passlimit);
    n.getParam("plane_detector/searchnumber",searchnumber);
    n.getParam("plane_detector/disp_point_number",disp_point_number);
    n.getParam("plane_detector/MinClusterSize",MinClusterSize);
    n.getParam("plane_detector/DistanceThreshold", DistanceThreshold);
    n.getParam("plane_detector/maxiter", maxiter);
    n.getParam("plane_detector/frame_id", frame_id);
    n.getParam("plane_detector/maxangle", maxangle);
    n.getParam("plane_detector/maxbaseangle",maxbaseangle);
    n.getParam("plane_detector/max_rel_angle",max_rel_angle);
    n.getParam("plane_detector/MinBaseSize",MinBaseSize);
    n.getParam("plane_detector/voxel",voxel);
    n.getParam("plane_detector/horizontal_publish_topic",horizontal_publish_topic);

    pub = n.advertise<sensor_msgs::PointCloud2> (map_publish_topic, 1);
    pub_marker = n.advertise<visualization_msgs::MarkerArray> ("/plane/markers", 10);
    pub_stair_h =n.advertise<sensor_msgs::PointCloud2> (horizontal_publish_topic, 1);
    pub_stair_rawh =n.advertise<sensor_msgs::PointCloud2> ("rawhorizontal", 1);
    // joint_pub  = n.advertise<sensor_msgs::JointState>("/compen_matrix", 5);
    sub = n.subscribe(pcl_subscribe_topic, 5, &Pdetect::cloud_cb,this);
    sub3 = n2.subscribe("/feet_PointCloud", 5, &Pdetect::feet_cloud_cb,this);

    baseplane.a=0.0;
    baseplane.b=0.0;
    baseplane.c=-1.0;
    baseplane.d=0.0;

    linecount=0;
    stair_clear=false;
    got_topic=false;
    stair_mapping_start=false;

}

Pdetect::~Pdetect(){

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planedetect");

  Pdetect pdetect;

  ros::spin();

  return 0;
}

