#include "../../include/zslam.h"

Zslam::Zslam(){

    map_publish_topic= "mapping_pcl";
    pcl_subscribe_topic= "/zed/point_cloud/cloud_registered";
    odom_subscribe_topic= "/zed/odom";
    leaf_size=0.1;
    disp_R=true;
    frame_id="map";

    n.getParam("zslam/map_publish_topic",map_publish_topic);
    n.getParam("zslam/pcl_subscribe_topic",pcl_subscribe_topic);
    n.getParam("zslam/odom_subscribe_topic",odom_subscribe_topic);
    n.getParam("zslam/leaf_size",leaf_size);
    n.getParam("zslam/disp_R",disp_R);
    n.getParam("zslam/frame_id",frame_id);

    //compensating
    n.getParam("zslam/linewidth",linewidth);//0.003
    n.getParam("zslam/bigradius",bigradius);//0.05
    n.getParam("zslam/mradius",mradius);//0.02
    n.getParam("zslam/compensationval",compensationval);//0.02

    pub = n.advertise<sensor_msgs::PointCloud2> (map_publish_topic, 1);
    pub1 = n.advertise<sensor_msgs::PointCloud2> ("intersectingline", 1);
    pub2 = n.advertise<std_msgs::Float64> ("alpha",1);
    pub3 = n.advertise<std_msgs::Float64> ("gamma",1);
    sub1 = n.subscribe(pcl_subscribe_topic, 10, &Zslam::pclCallback,this);
    sub2 = n.subscribe(odom_subscribe_topic, 10, &Zslam::zodomCallback,this);

    compensate_matrix.resize(4,4);
    compensate_matrix(0,0)=1.0;
    compensate_matrix(1,1)=1.0;
    compensate_matrix(2,2)=1.0;
    compensate_matrix(3,3)=1.0;
    compensate_matrix(1,3)=0.0;

    old_concave_exist=false;
    old_convex_exist=false;

    Con a;
    oldconcave_set.push_back(a);
    oldconvex_set.push_back(a);
    newconcave_set.push_back(a);
    newconvex_set.push_back(a);
    oldconcave_set.clear();
    oldconvex_set.clear();
    newconcave_set.clear();
    newconvex_set.clear();

    alpha=0.0;
    gamma=0.0;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "node");

  Zslam slamobject;

  ros::spin();

  return 0;
}
