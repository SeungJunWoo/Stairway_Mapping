#include "../../include/plane_detector.h"

int Pdetect::getPlaneModel(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr,
                           pcl::ModelCoefficients::Ptr coefficients_plane, int idx,
                           visualization_msgs::MarkerArray &markers)
{

    // compute principal direction
    Eigen::Vector4f centroid;
    Eigen::Matrix3f covariance;
    pcl::compute3DCentroid(*point_cloud_ptr, centroid);
    computeCovarianceMatrixNormalized(*point_cloud_ptr, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
    eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

    // move the points to the that reference frame
    Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
    Eigen::Matrix4f w2p(Eigen::Matrix4f::Identity());
    p2w.block<3, 3>(0, 0) = eigDx.transpose();
    p2w.block<3, 1>(0, 3) = -1.f * (p2w.block<3, 3>(0, 0) * centroid.head<3>());
    w2p = p2w.inverse();
    pcl::PointCloud<pcl::PointXYZ> cPoints;
    pcl::transformPointCloud(*point_cloud_ptr, cPoints, p2w);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);
    const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

    // final transform
    const Eigen::Quaternionf qfinal(eigDx);
    const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();


    double width = max_pt.z-min_pt.z; // longest line of cube
    double height= max_pt.y-min_pt.y;
    double depth = max_pt.x-min_pt.x; // shortest line of cube

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_ptr->resize(5);
    cloud_ptr->points[0] = pcl::PointXYZ(0, max_pt.y, max_pt.z);		// corner
    cloud_ptr->points[1] = pcl::PointXYZ(0, max_pt.y, min_pt.z);
    cloud_ptr->points[2] = pcl::PointXYZ(0, min_pt.y, max_pt.z);
    cloud_ptr->points[3] = pcl::PointXYZ(0, min_pt.y, min_pt.z);
    cloud_ptr->points[4] = pcl::PointXYZ(0.0, 0.0, 0.0);					  // center
    pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, w2p);



    ////////////////////////////////////   marker setting
    visualization_msgs::Marker marker_pt,marker_pts,marker_vector,marker_text;

    // Marker for corner points
    marker_pts.header.frame_id = "zed_center";
    marker_pts.header.stamp = ros::Time::now();
    geometry_msgs::Point pt[5];
    for(int i=0; i<5;i++)
    {
      pt[i].x=cloud_ptr->points[i].x/1.0f;
      pt[i].y=cloud_ptr->points[i].y/1.0f;
      pt[i].z=cloud_ptr->points[i].z/1.0f;
      if(i!=4)
            marker_pts.points.push_back(pt[i]);
    }
    marker_pts.ns = "corner_points";
    marker_pts.id = idx;
    marker_pts.type = visualization_msgs::Marker::POINTS;
    marker_pts.action = visualization_msgs::Marker::ADD;
    marker_pts.pose.position.x = 0.0;
    marker_pts.pose.position.y = 0.0;
    marker_pts.pose.position.z = 0.0;
    marker_pts.pose.orientation.x = 0.0;
    marker_pts.pose.orientation.y = 0.0;
    marker_pts.pose.orientation.z = 0.0;
    marker_pts.pose.orientation.w = 1.0;
    marker_pts.scale.x = 0.01f;
    marker_pts.scale.y = 0.01f;
    marker_pts.scale.z = 0.01f;
    marker_pts.color.r = 1.0f;
    marker_pts.color.g = 1.0f;
    marker_pts.color.b = 0.0f;
    marker_pts.color.a = 1.0;
    marker_pts.lifetime = ros::Duration(0.2);

    markers.markers.push_back(marker_pts);

    // Marker for center point
    marker_pt.header.frame_id = "zed_center";
    marker_pt.header.stamp = ros::Time::now();
    marker_pt.ns = "center_point";
    marker_pt.id = idx*100;
    marker_pt.type = visualization_msgs::Marker::SPHERE;
    marker_pt.action = visualization_msgs::Marker::ADD;
    marker_pt.pose.position.x = pt[4].x;
    marker_pt.pose.position.y = pt[4].y;
    marker_pt.pose.position.z = pt[4].z;
    marker_pt.pose.orientation.x = 0.0;
    marker_pt.pose.orientation.y = 0.0;
    marker_pt.pose.orientation.z = 0.0;
    marker_pt.pose.orientation.w = 1.0;
    marker_pt.scale.x = 0.01f;
    marker_pt.scale.y = 0.01f;
    marker_pt.scale.z = 0.01f;
    marker_pt.color.r = 1.0f;
    marker_pt.color.g = 1.0f;
    marker_pt.color.b = 0.0f;
    marker_pt.color.a = 1.0;
    marker_pt.lifetime = ros::Duration(0.2);

    markers.markers.push_back(marker_pt);

    // Marker for normal vector
    geometry_msgs::Point end_pt;
    end_pt.x =pt[4].x-coefficients_plane->values[0]/10.0;
    end_pt.y =pt[4].y-coefficients_plane->values[1]/10.0;
    end_pt.z =pt[4].z-coefficients_plane->values[2]/10.0;

    marker_vector.header.frame_id = "zed_center";
    marker_vector.header.stamp = ros::Time::now();
    marker_vector.ns = "normal_vector";
    marker_vector.id = idx*1000;
    marker_vector.type = visualization_msgs::Marker::ARROW;
    marker_vector.action = visualization_msgs::Marker::ADD;
    marker_vector.pose.position.x = 0.0;
    marker_vector.pose.position.y = 0.0;
    marker_vector.pose.position.z = 0.0;
    marker_vector.pose.orientation.x = 0.0;
    marker_vector.pose.orientation.y = 0.0;
    marker_vector.pose.orientation.z = 0.0;
    marker_vector.pose.orientation.w = 1.0;
    marker_vector.scale.x = 0.01f;
    marker_vector.scale.y = 0.01f;
    marker_vector.scale.z = 0.01f;
    marker_vector.color.r = 1.0f;
    marker_vector.color.g = 1.0f;
    marker_vector.color.b = 0.0f;
    marker_vector.color.a = 1.0;
    marker_vector.lifetime = ros::Duration(0.2);
    marker_vector.points.push_back(pt[4]);
    marker_vector.points.push_back(end_pt);

    markers.markers.push_back(marker_vector);

    // Marker for text visualization
    std::string msg="";
    std::stringstream ss;
    ss << width;
    msg += "Width: " + ss.str() + " m\n";
    ss.str("");
    ss << height;
    msg += "Height: " + ss.str() + " m";


    marker_text.header.frame_id = "zed_center";
    marker_text.header.stamp = ros::Time::now();
    marker_text.ns = "text";
    marker_text.id = idx*10000;
    marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_text.action = visualization_msgs::Marker::ADD;
    marker_text.pose.position.x = end_pt.x;
    marker_text.pose.position.y = end_pt.y;
    marker_text.pose.position.z = end_pt.z;
    marker_text.pose.orientation.x = 0.0;
    marker_text.pose.orientation.y = 0.0;
    marker_text.pose.orientation.z = 0.0;
    marker_text.pose.orientation.w = 1.0;
    marker_text.scale.x = 0.05f;
    marker_text.scale.y = 0.05f;
    marker_text.scale.z = 0.05f;
    marker_text.color.r = 0.0f;
    marker_text.color.g = 0.0f;
    marker_text.color.b = 0.0f;
    marker_text.color.a = 1.0;
    marker_text.lifetime = ros::Duration(0.2);
    marker_text.text = msg;
    markers.markers.push_back(marker_text);

    return(0);
  }
