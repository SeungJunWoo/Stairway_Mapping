#include "../../include/zslam.h"

void Zslam::zodomCallback(const nav_msgs::Odometry::ConstPtr& tfmsg)
{
  posx=tfmsg->pose.pose.position.x;
  posy=tfmsg->pose.pose.position.y;
  posz=tfmsg->pose.pose.position.z;

  orix=tfmsg->pose.pose.orientation.x;
  oriy=tfmsg->pose.pose.orientation.y;
  oriz=tfmsg->pose.pose.orientation.z;
  oriw=tfmsg->pose.pose.orientation.w;

  //cout<<orix<<" "<<oriy<<" "<<oriz<<" "<<oriw<<endl;

}


