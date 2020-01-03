
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>

// PCL specific includes
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/region_growing.h>
#include <iostream>
#include <fstream>
#include <boost/thread/thread.hpp>
#include <pcl/registration/icp.h>

#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <cmath>

#include <eigen3/Eigen/Dense>
#include <Eigen/Core>

#include <vector>
#include <map>
#include <algorithm>

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <eigen3/Eigen/Dense>

#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

using namespace Eigen;
using namespace std;

class Zslam
{
public:

    Zslam();
    void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& recived_pcl_msg);
    void zodomCallback(const nav_msgs::Odometry::ConstPtr& tfmsg);


    pcl::PointCloud<pcl::PointXYZ> pointcloud;

public:
    string map_publish_topic;
    string pcl_subscribe_topic;
    string odom_subscribe_topic;
    float leaf_size;
    bool disp_R;
    string frame_id;

    float linewidth;
    float bigradius;
    float mradius;
    float compensationval;

    struct Con
    {
        pcl::PointXYZ point;
        int count;
        float x_diff;
        float z_diff;
        int same_index;
        bool pri_existance;
        };



private:
    ros::NodeHandle n;
    ros::Publisher pub, pub1,pub2,pub3;
    ros::Subscriber sub1;
    ros::Subscriber sub2;

    float posx;
    float posy;
    float posz;
    float orix;
    float oriy;
    float oriz;
    float oriw;

    int frame;
    int stop_concave_count;
    int stop_convex_count;
    Eigen::MatrixXf compensate_matrix;

    pcl::PointXYZ keyconcave_Point;
    pcl::PointXYZ keyconvex_Point;
    pcl::PointXYZ newconcave_Point;
    pcl::PointXYZ newconvex_Point;
    pcl::PointXYZ oldconcave_Point;
    pcl::PointXYZ oldconvex_Point;

    vector<Con> keyconcave_set;
    vector<Con> keyconvex_set;
    vector<Con> newconcave_set;
    vector<Con> newconvex_set;
    vector<Con> oldconcave_set;
    vector<Con> oldconvex_set;

    float alpha;
    float gamma;

    bool old_concave_exist;
    bool old_convex_exist;

};
