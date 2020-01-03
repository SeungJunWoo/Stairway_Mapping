#include "../../include/robotodom2.h"

Robotodom2::Robotodom2(){

    pub = n.advertise<sensor_msgs::PointCloud2> ("Odomed_PointCloud", 1);
    sub1 = n.subscribe("filtered_PointCloud", 2, &Robotodom2::pclCallback,this);

    pub2 = n2.advertise<sensor_msgs::PointCloud2> ("feet_PointCloud", 1);
    sub2 = n2.subscribe("fl", 2, &Robotodom2::flCallback,this);
    sub3 = n3.subscribe("fr", 2, &Robotodom2::frCallback,this);
    sub4 = n4.subscribe("bl", 2, &Robotodom2::blCallback,this);
    sub5 = n5.subscribe("br", 2, &Robotodom2::brCallback,this);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotodom2");

  Robotodom2 slamobject;

  ros::spin();

  return 0;
}
