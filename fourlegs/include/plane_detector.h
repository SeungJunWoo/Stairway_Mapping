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
#include <pcl/filters/project_inliers.h>
#include "sensor_msgs/JointState.h"


#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <cmath>

#include <eigen3/Eigen/Dense>

#include <vector>
#include <algorithm>



using namespace Eigen;
using namespace std;

class Pdetect
{
public:
    string map_publish_topic;
    string pcl_subscribe_topic;
    string odom_subscribe_topic;
    string fieldname;
    string horizontal_publish_topic;
    string vertical_publish_topic;
    float passlimit;
    int searchnumber;
    bool disp_point_number;
    int MinClusterSize;
    int MinBaseSize;
    double DistanceThreshold;
    int maxiter;
    string frame_id;
    int maxangle;
    int maxbaseangle;
    int max_rel_angle;
    float voxel;
    int ksearch;
    float NormalDistanceWeight;
    float weight;
    bool stair_clear;
    bool got_topic;
    bool stair_mapping_start;
    bool plane_color;

    typedef pcl::PointXYZ PointT;
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    pcl::PointCloud<pcl::PointXYZ> footcloud;

    struct Plane
    {
        float a;
        float b;
        float c;
        float d;
        float var_d;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud;
	    bool pointexist;

        Matrix3f covariance;
    };

    struct Footpoint
    {
        pcl::PointXYZ point;
        int index;
        bool inlier;

    };

    struct Step
    {
        vector<Plane> inlier_planes;
        Plane key_stair;
        Plane key_stair_ref;
        Plane measurement;
        Plane estimation;
        Eigen::Vector4f centroid;
    };


public:

    Pdetect();
    ~Pdetect();
    void cloud_cb (const sensor_msgs::PointCloud2::ConstPtr& recived_pcl_msg);
    void feet_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr &recived_pcl_msg);

    void addPointColoudToColouredPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane,
                 pcl::PointCloud<pcl::PointXYZI> &coloured_point_cloud, int idx, bool tmp);

    int getPlaneModel(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr,
                 pcl::ModelCoefficients::Ptr coefficients_plane, int idx,
                      visualization_msgs::MarkerArray &markers);
    void zodomCallback(const nav_msgs::Odometry::ConstPtr& tfmsg);
    float get_planeangle(float a, float b, float c, float d);
    float get_relative_angle(Plane pri, Plane newplane);
    void plane_transform();
    Plane ave(std::vector<Plane> &a);
    Plane esti(std::vector<Step> &a);
    Plane esti_withkey(std::vector<Step> &a);
    float m_var(std::vector<Plane> &a,Plane average);
    Matrix3f m_covar(std::vector<Plane> &a,Plane average);
    Matrix3f e_covar(std::vector<Step> &a,Plane average);
    void step_modeling_horizontal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane,float a, float b, float c, float d);
    void step_modeling_horizontal2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane,float a, float b, float c, float d);
    // void step_modeling_vertical(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane,float a, float b, float c, float d);

private:
    ros::NodeHandle n,n2;
    ros::Publisher pub,pub_marker,pub_stair_h,pub_stair_v,pub_stair_rawh,joint_pub;
    ros::Subscriber sub;
    ros::Subscriber sub2,sub3;

    Plane baseplane;
    //Plane t_plane;//del

    //float planeangle;//del

    ofstream myfile;
    int linecount;

    vector<Plane> plane_from_feet;
    vector<Step> stair;
    vector<Step> stair2;

    vector<Plane> key_stair_v;
    vector<Plane> key_stair_ref_v;
    vector<Plane> measurement_v;
    vector<Plane> estimation_v;
    vector<vector<Plane> > stair_v;

};
