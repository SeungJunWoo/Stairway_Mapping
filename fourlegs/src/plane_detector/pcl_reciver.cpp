#include "../../include/plane_detector.h"

void Pdetect::cloud_cb(const sensor_msgs::PointCloud2::ConstPtr &recived_pcl_msg)
{

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*recived_pcl_msg, *cloud);

    //    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);

    int i = 0, nr_points = (int)cloud->points.size();
    int plane_found = 0;
    int csize = 0;
    if (disp_point_number)
    {
        std::cout << "number of points =" << nr_points << std::endl;
    }

    //Segmentation part/////////////////////////////

    //segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    //extraction object
    pcl::ExtractIndices<PointT> extract;
    //pcl::ExtractIndices<pcl::Normal> extract_normals;

    //color object
    pcl::PointCloud<pcl::PointXYZI> coloured_point_cloud;

    //visualization object
    visualization_msgs::MarkerArray markers;

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold(DistanceThreshold);

    pcl::PointCloud<pcl::PointXYZ> cloud_plane2;

    while (cloud->points.size() > 0.3 * nr_points)
    {
        pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
        seg.setInputCloud(cloud);
        seg.segment(*inliers_plane, *coefficients_plane);

        if (inliers_plane->indices.size() == 0)
        {
            break;
        }

        // Extract the planar inliers from the input cloud
        extract.setInputCloud(cloud);
        extract.setIndices(inliers_plane);
        extract.setNegative(false);
        extract.filter(*cloud_plane);

        //detected plane visualization
        for (int i = 0; i < cloud_plane->size(); i++)
        {
            pcl::PointXYZ pt = cloud_plane->points[i];

            pcl::PointXYZ temp_point;
            temp_point.x = pt.x;
            temp_point.y = pt.y;
            temp_point.z = pt.z;
            cloud_plane2.push_back(temp_point);
        }

        bool haveMinSize = true;
        csize = (int)(inliers_plane->indices.size());

        if (csize < MinClusterSize)
            haveMinSize = false;
        if (haveMinSize == false /*accepted==-1*/)
        {
        }
        else
        {

            float planeangle;
            float a1;
            float b1;
            float c1;
            float d1;
            a1 = coefficients_plane->values[0];
            b1 = coefficients_plane->values[1];
            c1 = coefficients_plane->values[2];
            d1 = coefficients_plane->values[3];

            planeangle = get_planeangle(a1, b1, c1, d1);

            if (planeangle < maxangle)
            {
                plane_found++;

                //getPlaneModel(cloud_plane, coefficients_plane,plane_found, markers);

                step_modeling_horizontal(cloud_plane, a1, b1, c1, d1);
            }
            else if (90 - 3 * maxangle < planeangle && planeangle < 90 + 3 * maxangle)
            {

                plane_found++;

                //getPlaneModel(cloud_plane, coefficients_plane,plane_found, markers);

                // step_modeling_vertical(cloud_plane, a1, b1, c1, d1);
            }
        }

        //        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_filtered2);

        cloud.swap(cloud_filtered2);

    } // while

    //stair found assosiation horizontal
    pcl::PointCloud<pcl::PointXYZRGB> horizontal;
    for (std::size_t i = 0; i != stair.size(); i++)
    {
        cout << "key_plane_ref_size: " << stair.size() << endl;
        cout << "measurement " << i << " :" << stair[i].measurement.a << "," << stair[i].measurement.b
             << "," << stair[i].measurement.c << "," << stair[i].measurement.d << " var: " << stair[i].measurement.var_d << endl;
        cout << "estimation " << i << " :" << stair[i].estimation.a << "," << stair[i].estimation.b
             << "," << stair[i].estimation.c << "," << stair[i].estimation.d << " var: " << stair[i].estimation.var_d << endl;
        cout << "key_plane " << i << " :" << stair[i].key_stair.a << "," << stair[i].key_stair.b
             << "," << stair[i].key_stair.c << "," << stair[i].key_stair.d << endl;
        cout << " size : " << stair[i].inlier_planes.size() << endl;
        //cout << stair[i].estimation.covariance<<endl;

        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*stair[i].key_stair.pointcloud, min_pt, max_pt);

        if (footcloud.size() == 4)
        {
            for (int k = 0; k < footcloud.size(); k++)
            {
                if (abs(stair[i].centroid[2] - footcloud.points[k].z) < 0.05 && min_pt.x < footcloud.points[k].x && max_pt.x > footcloud.points[k].x)
                {
                    plane_color = false; //red
                    float distance_feet = stair[i].key_stair.a * footcloud.points[k].x + 
                        stair[i].key_stair.b * footcloud.points[k].y + 
                        stair[i].key_stair.c * footcloud.points[k].z + 
                        stair[i].key_stair.d;
                    if (distance_feet <0){
                        cout<<"distance feet _______________________"<<distance_feet<<endl;

                    }                    
                    break;
                }
                else
                {
                    plane_color = true; //blue
                }
            }
        }

        else
        {
            plane_color = true;
        }

        if (plane_color == false)
        {
            for (size_t j = 0; j < stair[i].key_stair.pointcloud->size(); ++j)
            {
                pcl::PointXYZRGB temp_point;
                // pcl::PointXYZ temp_point;
                temp_point.x = stair[i].key_stair.pointcloud->points[j].x;
                temp_point.y = stair[i].key_stair.pointcloud->points[j].y;
                temp_point.z = stair[i].key_stair.pointcloud->points[j].z;
                temp_point.r = 250;
                temp_point.g = 128;
                temp_point.b = 114;
                horizontal.push_back(temp_point);
            }
        }
        else
        {
            for (size_t j = 0; j < stair[i].key_stair.pointcloud->size(); ++j)
            {
                pcl::PointXYZRGB temp_point;
                // pcl::PointXYZ temp_point;
                temp_point.x = stair[i].key_stair.pointcloud->points[j].x;
                temp_point.y = stair[i].key_stair.pointcloud->points[j].y;
                temp_point.z = stair[i].key_stair.pointcloud->points[j].z;
                temp_point.r = 135;
                temp_point.g = 206;
                temp_point.b = 235;
                horizontal.push_back(temp_point);
            }
        }
    }


    for (std::size_t i = 0; i != stair2.size(); i++)
    {
        cout << "key_plane_ref_size: " << stair2.size() << endl;
        cout << "measurement " << i << " :" << stair2[i].measurement.a << "," << stair2[i].measurement.b
             << "," << stair2[i].measurement.c << "," << stair2[i].measurement.d << " var: " << stair2[i].measurement.var_d << endl;
        cout << "estimation " << i << " :" << stair2[i].estimation.a << "," << stair2[i].estimation.b
             << "," << stair2[i].estimation.c << "," << stair2[i].estimation.d << " var: " << stair2[i].estimation.var_d << endl;
        cout << "key_plane " << i << " :" << stair2[i].key_stair.a << "," << stair2[i].key_stair.b
             << "," << stair2[i].key_stair.c << "," << stair2[i].key_stair.d << endl;
        cout << " size : " << stair2[i].inlier_planes.size() << endl;
        // cout << stair[i].estimation.covariance<<endl;

        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*stair2[i].key_stair.pointcloud, min_pt, max_pt);

        if (footcloud.size() == 4)
        {
            for (int k = 0; k < footcloud.size(); k++)
            {
                if (abs(stair2[i].centroid[2] - footcloud.points[k].z) < 0.05 && min_pt.x < footcloud.points[k].x && max_pt.x > footcloud.points[k].x)
                {
                    plane_color = false; //red
                    break;
                }
                else
                {
                    plane_color = true; //blue
                }
            }
        }

        else
        {
            plane_color = true;
        }

        if (plane_color == false)
        {
            for (size_t j = 0; j < stair2[i].key_stair.pointcloud->size(); ++j)
            {
                pcl::PointXYZRGB temp_point;
                // pcl::PointXYZ temp_point;
                temp_point.x = stair2[i].key_stair.pointcloud->points[j].x;
                temp_point.y = stair2[i].key_stair.pointcloud->points[j].y;
                temp_point.z = stair2[i].key_stair.pointcloud->points[j].z;
                temp_point.r = 250;
                temp_point.g = 128;
                temp_point.b = 114;
                horizontal.push_back(temp_point);
            }
        }
        else
        {
            for (size_t j = 0; j < stair2[i].key_stair.pointcloud->size(); ++j)
            {
                pcl::PointXYZRGB temp_point;
                // pcl::PointXYZ temp_point;
                temp_point.x = stair2[i].key_stair.pointcloud->points[j].x;
                temp_point.y = stair2[i].key_stair.pointcloud->points[j].y;
                temp_point.z = stair2[i].key_stair.pointcloud->points[j].z;
                temp_point.r = 135;
                temp_point.g = 206;
                temp_point.b = 235;
                horizontal.push_back(temp_point);
            }
        }
    }

    if (stair_mapping_start == true)
    {
        cout << endl;
    }

    // Convert To ROS data type (all plane)
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(cloud_plane2, cloud_p);

    pcl_conversions::fromPCL(cloud_p, output);
    output.header.frame_id = frame_id;
    pub.publish(output);

    // Convert To ROS data type (horizontal plane)
    sensor_msgs::PointCloud2 output2;
    pcl::PCLPointCloud2 cloud_p2;
    pcl::toPCLPointCloud2(horizontal, cloud_p2);

    pcl_conversions::fromPCL(cloud_p2, output2);
    output2.header.frame_id = frame_id;
    pub_stair_h.publish(output2);

    // Publish markers
    pub_marker.publish(markers);
}
