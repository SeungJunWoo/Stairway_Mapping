#include "../../include/plane_detector.h"

void Pdetect::step_modeling_horizontal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane, float a, float b, float c, float d)
{

    Plane t_plane;
    t_plane.a = a;
    t_plane.b = b;
    t_plane.c = c;
    t_plane.d = d;
    t_plane.pointcloud = cloud_plane;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_plane, centroid);
    // cout<<"centroid : "<<centroid[0]<<","<<centroid[1]<<","<<centroid[2]<<endl;

    if (stair.size() == 0)
    {
        Step step;
        step.key_stair = t_plane;
        t_plane.pointexist = false;
        step.key_stair_ref = t_plane;
        step.measurement = t_plane;
        step.estimation = t_plane;
        step.centroid = centroid;
        stair.push_back(step);
    }

    bool inlier = false;
    bool noise = false;
    for (auto &elem : stair)
    {
        // Plane pos = elem;
        int i = &elem - &stair[0];
        float rel_angle;
        rel_angle = get_relative_angle(elem.key_stair, t_plane);
        cout << "rel_angle : " << rel_angle << endl;

        if ((abs(centroid[2] - elem.centroid[2]) < 0.05) && (rel_angle < max_rel_angle))
        {
            //cout << "keystair i : " << i << endl;
            float center_offset;
            center_offset = sqrt(pow((elem.centroid[0] - centroid[0]), 2.0) + pow((elem.centroid[1] - centroid[1]), 2.0) + pow((elem.centroid[2] - centroid[2]), 2.0));
            // center_offset = abs(elem.centroid[0]-centroid[0]);
            cout << "center offset : " << center_offset << endl;
            if (center_offset > 0.6)
            {
                inlier = true;
                step_modeling_horizontal2(cloud_plane, a, b, c, d);
                break;
            }
            else
            {

                inlier = true;

                stair[i].inlier_planes.push_back(t_plane);

                if (stair[i].inlier_planes.size() > 10000)
                {
                    stair[i].inlier_planes.erase(stair[i].inlier_planes.begin());
                }

                stair[i].measurement = ave(stair[i].inlier_planes);

                stair[i].measurement.var_d = m_var(stair[i].inlier_planes, stair[i].measurement);        //covariance of d
                stair[i].measurement.covariance = m_covar(stair[i].inlier_planes, stair[i].measurement); //covariance of a b c

                if (i < 3)
                {
                    stair[i].estimation = stair[i].measurement;
                    stair[i].estimation.var_d = 0;
                }
                else if (i > 2)
                {
                    stair[i].estimation = esti(stair); // case 1
                    stair[i].estimation.d = stair[i - 1].measurement.d - stair[i].estimation.d;
                    stair[i].estimation.covariance = e_covar(stair, stair[i].estimation);

                    // stair[i].estimation = esti_withkey(stair); // case 2
                    // stair[i].estimation.d = stair[i - 1].key_stair.d - stair[i].estimation.d;
                    // stair[i].estimation.covariance = e_covar(stair, stair[i].estimation);
                }

                //fusing key plane

                float e_var = stair[i].estimation.var_d;  //for d only
                float m_var = stair[i].measurement.var_d; // for d only

                if (i < 3)
                {

                    stair[i].key_stair_ref.a = stair[i].measurement.a;
                    stair[i].key_stair_ref.b = stair[i].measurement.b;
                    stair[i].key_stair_ref.c = stair[i].measurement.c;
                    stair[i].key_stair_ref.d = stair[i].measurement.d;
                }
                else if (i > 2)
                {
                    //a b c
                    MatrixXf m(3, 1);
                    m(0, 0) = stair[i].measurement.a;
                    m(1, 0) = stair[i].measurement.b;
                    m(2, 0) = stair[i].measurement.c;
                    MatrixXf e(3, 1);
                    e(0, 0) = stair[i].estimation.a;
                    e(1, 0) = stair[i].estimation.b;
                    e(2, 0) = stair[i].estimation.c;
                    Matrix3f Km;
                    Matrix3f Ke;

                    Km = stair[i].estimation.covariance * (stair[i].measurement.covariance + stair[i].estimation.covariance).inverse();
                    Ke = stair[i].measurement.covariance * (stair[i].measurement.covariance + stair[i].estimation.covariance).inverse();
                    MatrixXf x(3, 1);
                    x = Km * m + Ke * e;
                    stair[i].key_stair_ref.a = x(0, 0);
                    stair[i].key_stair_ref.b = x(1, 0);
                    stair[i].key_stair_ref.c = x(2, 0);

                    float weight2 = e_var / (m_var + e_var);
                    cout << "weight : " << weight2 << endl;
                    stair[i].key_stair_ref.d = weight2 * stair[i].measurement.d + (1 - weight2) * stair[i].estimation.d;
                }
            }

            // pointcloud
            if (stair[i].key_stair_ref.pointexist == false)
            {
                stair[i].key_stair_ref.pointcloud = cloud_plane;
                stair[i].key_stair_ref.pointexist = true;
            }
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
            coefficients->values.resize(4);
            coefficients->values[0] = stair[i].key_stair_ref.a;
            coefficients->values[1] = stair[i].key_stair_ref.b;
            coefficients->values[2] = stair[i].key_stair_ref.c;
            coefficients->values[3] = stair[i].key_stair_ref.d;

            stair[i].key_stair.a = stair[i].key_stair_ref.a;
            stair[i].key_stair.b = stair[i].key_stair_ref.b;
            stair[i].key_stair.c = stair[i].key_stair_ref.c;
            stair[i].key_stair.d = stair[i].key_stair_ref.d;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected_set(new pcl::PointCloud<pcl::PointXYZ>);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ProjectInliers<pcl::PointXYZ> proj;
            proj.setModelType(pcl::SACMODEL_PLANE);
            proj.setInputCloud(cloud_plane);
            proj.setModelCoefficients(coefficients);
            proj.filter(*cloud_projected);

            for (size_t k = 0; k < cloud_projected->size(); ++k)
            {
                pcl::PointXYZ temp_point;
                temp_point.x = cloud_projected->points[k].x;
                temp_point.y = cloud_projected->points[k].y;
                temp_point.z = cloud_projected->points[k].z;
                stair[i].key_stair_ref.pointcloud->points.push_back(temp_point);
                //cloud_projected_set->points.push_back(temp_point);
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected2(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ProjectInliers<pcl::PointXYZ> proj2;
            proj2.setModelType(pcl::SACMODEL_PLANE);
            proj2.setInputCloud(stair[i].key_stair_ref.pointcloud);
            proj2.setModelCoefficients(coefficients);
            proj2.filter(*cloud_projected2);
            for (size_t k = 0; k < cloud_projected2->size(); ++k)
            {
                pcl::PointXYZ temp_point;
                temp_point.x = cloud_projected2->points[k].x;
                temp_point.y = cloud_projected2->points[k].y;
                temp_point.z = cloud_projected2->points[k].z;
                cloud_projected_set->points.push_back(temp_point);
            }

            // voxel filtering
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected_set_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> vox;
            vox.setInputCloud(cloud_projected_set);
            vox.setLeafSize(voxel, voxel, voxel);
            vox.filter(*cloud_projected_set_filtered);

            stair[i].key_stair.pointcloud = cloud_projected_set_filtered;

            Eigen::Vector4f centroid_k;
            pcl::compute3DCentroid(*stair[i].key_stair.pointcloud, centroid_k);
            stair[i].centroid = centroid_k;

            break;
        }
        else if (abs(centroid[2] - elem.centroid[2]) > 0.05 && abs(centroid[2] - elem.centroid[2]) < 0.10)
        {
            noise = true;
        }
        else if (rel_angle > max_rel_angle)
        {
            noise = true;
        }
    }

    float rel_angle;
    rel_angle = get_relative_angle(stair.back().key_stair, t_plane);

    if (inlier == false && (centroid[2] - stair.back().centroid[2]) > 0.10 && rel_angle < max_rel_angle)
    {
        cout << "t_plane d :" << t_plane.d << " key_stair.back.d :" << stair.back().key_stair.d << endl;
        Step temp_step;
        temp_step.measurement = t_plane;
        t_plane.pointexist = false;
        temp_step.estimation = t_plane;
        temp_step.key_stair = t_plane;
        temp_step.key_stair_ref = t_plane;
        temp_step.centroid = centroid;
        temp_step.inlier_planes.push_back(t_plane);

        stair.push_back(temp_step);
    }
    if (inlier == false && (stair.back().centroid[2] - centroid[2]) > 0.10 && noise == false && rel_angle < max_rel_angle)
    {
        cout << "t_plane d :" << t_plane.d << " key_stair.back.d :" << stair.back().key_stair.d << endl;
        Step temp_step;
        temp_step.measurement = t_plane;
        t_plane.pointexist = false;
        temp_step.estimation = t_plane;
        temp_step.key_stair = t_plane;
        temp_step.key_stair_ref = t_plane;
        temp_step.centroid = centroid;
        temp_step.inlier_planes.push_back(t_plane);

        for (std::size_t i = 0; i != stair.size(); i++)
        {
            if (stair[i].measurement.d < t_plane.d)
            {
                stair.insert(stair.begin() + i, temp_step);

                break;
            }
        }
    }
}

void Pdetect::step_modeling_horizontal2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane, float a, float b, float c, float d)
{

    Plane t_plane;
    t_plane.a = a;
    t_plane.b = b;
    t_plane.c = c;
    t_plane.d = d;
    t_plane.pointcloud = cloud_plane;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_plane, centroid);

    if (stair2.size() == 0)
    {
        Step step;
        step.key_stair = t_plane;
        t_plane.pointexist = false;
        step.key_stair_ref = t_plane;
        step.measurement = t_plane;
        step.estimation = t_plane;
        step.centroid = centroid;
        stair2.push_back(step);
    }

    bool inlier = false;
    bool noise = false;
    for (auto &elem : stair2)
    {
        // Plane pos = elem;
        int i = &elem - &stair2[0];
        float rel_angle;
        rel_angle = get_relative_angle(elem.key_stair, t_plane);
        cout << "rel_angle : " << rel_angle << endl;

        if ((abs(t_plane.d - elem.key_stair.d) < 0.07) && (rel_angle < 1.5 * max_rel_angle))
        {

            inlier = true;

            stair2[i].inlier_planes.push_back(t_plane);

            if (stair2[i].inlier_planes.size() > 100)
            {
                stair2[i].inlier_planes.erase(stair2[i].inlier_planes.begin());
            }

            stair2[i].measurement = ave(stair2[i].inlier_planes);

            stair2[i].measurement.var_d = m_var(stair2[i].inlier_planes, stair2[i].measurement);        //covariance of d
            stair2[i].measurement.covariance = m_covar(stair2[i].inlier_planes, stair2[i].measurement); //covariance of a b c

            if (i < 3)
            {
                stair2[i].estimation = stair2[i].measurement;
                stair2[i].estimation.var_d = 0;
            }
            else if (i > 2)
            {
                stair2[i].estimation = esti(stair2);
                stair2[i].estimation.d = stair2[i - 1].measurement.d - stair2[i].estimation.d;
                stair2[i].estimation.covariance = e_covar(stair2, stair2[i].estimation);
            }

            //fusing key plane

            float e_var = stair2[i].estimation.var_d;  //for d only
            float m_var = stair2[i].measurement.var_d; // for d only

            if (i < 3)
            {

                stair2[i].key_stair_ref.a = stair2[i].measurement.a;
                stair2[i].key_stair_ref.b = stair2[i].measurement.b;
                stair2[i].key_stair_ref.c = stair2[i].measurement.c;
                stair2[i].key_stair_ref.d = stair2[i].measurement.d;
            }
            else if (i > 2)
            {

                //a b c
                MatrixXf m(3, 1);
                m(0, 0) = stair2[i].measurement.a;
                m(1, 0) = stair2[i].measurement.b;
                m(2, 0) = stair2[i].measurement.c;
                MatrixXf e(3, 1);
                e(0, 0) = stair2[i].estimation.a;
                e(1, 0) = stair2[i].estimation.b;
                e(2, 0) = stair2[i].estimation.c;
                Matrix3f Km;
                Matrix3f Ke;

                Km = stair2[i].estimation.covariance * (stair2[i].measurement.covariance + stair2[i].estimation.covariance).inverse();
                Ke = stair2[i].measurement.covariance * (stair2[i].measurement.covariance + stair2[i].estimation.covariance).inverse();
                MatrixXf x(3, 1);
                x = Km * m + Ke * e;
                stair2[i].key_stair_ref.a = x(0, 0);
                stair2[i].key_stair_ref.b = x(1, 0);
                stair2[i].key_stair_ref.c = x(2, 0);

                float weight2 = e_var / (m_var + e_var);
                cout << "weight : " << weight2 << endl;
                stair2[i].key_stair_ref.d = weight2 * stair2[i].measurement.d + (1 - weight2) * stair2[i].estimation.d;
            }

            // pointcloud
            if (stair2[i].key_stair_ref.pointexist == false)
            {
                stair2[i].key_stair_ref.pointcloud = cloud_plane;
                stair2[i].key_stair_ref.pointexist = true;
            }
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
            coefficients->values.resize(4);
            coefficients->values[0] = stair2[i].key_stair_ref.a;
            coefficients->values[1] = stair2[i].key_stair_ref.b;
            coefficients->values[2] = stair2[i].key_stair_ref.c;
            coefficients->values[3] = stair2[i].key_stair_ref.d;

            stair2[i].key_stair.a = stair2[i].key_stair_ref.a;
            stair2[i].key_stair.b = stair2[i].key_stair_ref.b;
            stair2[i].key_stair.c = stair2[i].key_stair_ref.c;
            stair2[i].key_stair.d = stair2[i].key_stair_ref.d;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected_set(new pcl::PointCloud<pcl::PointXYZ>);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ProjectInliers<pcl::PointXYZ> proj;
            proj.setModelType(pcl::SACMODEL_PLANE);
            proj.setInputCloud(cloud_plane);
            proj.setModelCoefficients(coefficients);
            proj.filter(*cloud_projected);

            for (size_t k = 0; k < cloud_projected->size(); ++k)
            {
                pcl::PointXYZ temp_point;
                temp_point.x = cloud_projected->points[k].x;
                temp_point.y = cloud_projected->points[k].y;
                temp_point.z = cloud_projected->points[k].z;
                stair2[i].key_stair_ref.pointcloud->points.push_back(temp_point);
                //cloud_projected_set->points.push_back(temp_point);
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected2(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ProjectInliers<pcl::PointXYZ> proj2;
            proj2.setModelType(pcl::SACMODEL_PLANE);
            proj2.setInputCloud(stair2[i].key_stair_ref.pointcloud);
            proj2.setModelCoefficients(coefficients);
            proj2.filter(*cloud_projected2);
            for (size_t k = 0; k < cloud_projected2->size(); ++k)
            {
                pcl::PointXYZ temp_point;
                temp_point.x = cloud_projected2->points[k].x;
                temp_point.y = cloud_projected2->points[k].y;
                temp_point.z = cloud_projected2->points[k].z;
                cloud_projected_set->points.push_back(temp_point);
            }

            // voxel filtering
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected_set_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> vox;
            vox.setInputCloud(cloud_projected_set);
            vox.setLeafSize(voxel, voxel, voxel);
            vox.filter(*cloud_projected_set_filtered);

            stair2[i].key_stair.pointcloud = cloud_projected_set_filtered;
            // pcl::PointXYZ min_pt, max_pt;
            // pcl::getMinMax3D(*stair2[i].key_stair.pointcloud, min_pt, max_pt);
            // double depth = max_pt.x - min_pt.x;

            //cout << "depth : " << depth << " key stair size : " << key_stair.size() << endl;
            // if (depth > 0.5 && stair2.size() > 2)
            // {
            //     stair_clear = true;
            // }

            Eigen::Vector4f centroid_k;
            pcl::compute3DCentroid(*stair2[i].key_stair.pointcloud, centroid_k);
            stair2[i].centroid = centroid_k;

            break;
        }
        else if (abs(t_plane.d - stair2[i].key_stair.d) > 0.07 && abs(t_plane.d - stair2[i].key_stair.d) < 0.10)
        {
            noise = true;
        }
    }

    float rel_angle;
    rel_angle = get_relative_angle(stair2.back().key_stair, t_plane);

    if (inlier == false && (t_plane.d - stair2.back().key_stair.d) < -0.10 && rel_angle < max_rel_angle)
    {
        cout << "t_plane d :" << t_plane.d << " key_stair.back.d :" << stair2.back().key_stair.d << endl;
        Step temp_step;
        temp_step.measurement = t_plane;
        t_plane.pointexist = false;
        temp_step.estimation = t_plane;
        temp_step.key_stair = t_plane;
        temp_step.key_stair_ref = t_plane;
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_plane, centroid);
        temp_step.centroid = centroid;
        temp_step.inlier_planes.push_back(t_plane);

        stair2.push_back(temp_step);
    }
    if (inlier == false && (t_plane.d - stair2.back().key_stair.d) > 0.10 && noise == false && rel_angle < max_rel_angle)
    {
        cout << "t_plane d :" << t_plane.d << " key_stair.back.d :" << stair2.back().key_stair.d << endl;
        Step temp_step;
        temp_step.measurement = t_plane;
        t_plane.pointexist = false;
        temp_step.estimation = t_plane;
        temp_step.key_stair = t_plane;
        temp_step.key_stair_ref = t_plane;
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_plane, centroid);
        temp_step.centroid = centroid;
        temp_step.inlier_planes.push_back(t_plane);

        for (std::size_t i = 0; i != stair2.size(); i++)
        {
            if (stair2[i].measurement.d < t_plane.d)
            {
                stair2.insert(stair2.begin() + i, temp_step);

                break;
            }
        }
    }
}
