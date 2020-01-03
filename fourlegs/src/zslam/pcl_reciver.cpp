#include "../../include/zslam.h"

void Zslam::pclCallback(const sensor_msgs::PointCloud2::ConstPtr &recived_pcl_msg)
{
    frame++;
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*recived_pcl_msg, *cloud);

    // passthrough filter to remove spurious NaNs
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-1.5, 1.5);
    pass.filter(*cloud_voxel);

    // voxel filtering
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(cloud_voxel);
    vox.setLeafSize(leaf_size, leaf_size, leaf_size);
    vox.filter(*cloud_filtered);

    //R0 //world to t265_odom_frame
    Quaternionf q0;
    q0.x() = 0.0;
    q0.y() = 0.0;
    q0.z() = -0.7071068;
    q0.w() = 0.7071068;
    MatrixXf R0 = q0.normalized().toRotationMatrix();

    R0.conservativeResize(4, 4);
    R0.row(3).setZero();
    R0(0, 3) = 0.0;
    R0(1, 3) = 0.0;
    R0(2, 3) = 0.0;
    R0(3, 3) = 1.0;

    //quaternion to rotation m
    Quaternionf q;
    q.x() = orix;
    q.y() = oriy;
    q.z() = oriz;
    q.w() = oriw;

    MatrixXf p_R = q.normalized().toRotationMatrix();
    p_R.conservativeResize(4, 4);
    p_R.row(3).setZero();
    p_R(0, 3) = posx;
    p_R(1, 3) = posy;
    p_R(2, 3) = posz;
    p_R(3, 3) = 1.0;

    MatrixXf R = p_R;
    R(1, 3) = posy + alpha;
    R(2, 3) = posz + gamma;

    //d435 //t265 to d435

    Quaternionf q2;
    q2.x() = 0.0;
    q2.y() = 1.0;
    q2.z() = 0.0;
    q2.w() = 0.0;
    MatrixXf R2 = q2.normalized().toRotationMatrix();

    R2.conservativeResize(4, 4);
    R2.row(3).setZero();
    R2(0, 3) = -0.085;
    R2(1, 3) = 0.0;
    R2(2, 3) = 0.0;
    R2(3, 3) = 1.0;

    //pcl //d435 to pcl

    Quaternionf q3;
    q3.x() = 0.5;
    q3.y() = -0.5;
    q3.z() = 0.5;
    q3.w() = -0.5;
    MatrixXf R3 = q3.normalized().toRotationMatrix();

    R3.conservativeResize(4, 4);
    R3.row(3).setZero();
    R3(0, 3) = 0.0;
    R3(1, 3) = 0.0;
    R3(2, 3) = 0.0;
    R3(3, 3) = 1.0;

    MatrixXf T(4, 4);
    T = R0 * R * R2 * R3;

    pcl::PointCloud<pcl::PointXYZ> temp_Cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr middle_Cloud_pointer(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr middle_Color_pointer(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
    {
        MatrixXf X(4, 1);
        X(0, 0) = cloud_filtered->points[i].x;
        X(1, 0) = cloud_filtered->points[i].y;
        X(2, 0) = cloud_filtered->points[i].z;
        X(3, 0) = 1.0;

        //cout<<X;

        MatrixXf Y(4, 1);
        Y = T * X;

        pcl::PointXYZ temp_point;
        temp_point.x = Y(0, 0);
        temp_point.y = Y(1, 0);
        temp_point.z = Y(2, 0);

        if (temp_point.y < -0.25+linewidth && temp_point.y > -0.25-linewidth)
        {
            pcl::PointXYZRGB color_temp_point;
            color_temp_point.x = temp_point.x;
            color_temp_point.y = temp_point.y;
            color_temp_point.z = temp_point.z;
            color_temp_point.b = 250;
            middle_Cloud_pointer->push_back(temp_point);
            middle_Color_pointer->push_back(color_temp_point);
        }

        //pushback
        temp_Cloud.push_back(temp_point);
        //pointcloud.push_back(temp_point);
    }

    //finding concave convex point
    vector<pcl::PointXYZ> concave;
    vector<Con> concave_set;
    vector<pcl::PointXYZ> convex;
    vector<Con> convex_set;

    for (size_t i = 0; i < middle_Cloud_pointer->points.size(); ++i)
    {

        bool up;
        bool down;
        bool front;
        bool back;
        bool farup;
        bool fardown;
        bool farfront;
        bool farback;
        bool rup;
        bool rdown;
        bool rfront;
        bool rback;
        up = false;
        down = false;
        front = false;
        back = false;
        farup = false;
        fardown = false;
        farfront = false;
        farback = false;
        rup = false;
        rdown = false;
        rfront = false;
        rback = false;

        pcl::PointXYZ searchPoint;

        searchPoint.x = middle_Cloud_pointer->points[i].x;
        searchPoint.y = middle_Cloud_pointer->points[i].y;
        searchPoint.z = middle_Cloud_pointer->points[i].z;

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

        kdtree.setInputCloud(middle_Cloud_pointer);

        std::vector<int> pointIdxRadius;
        std::vector<float> pointsSquaredDistRadius;
        float radius = bigradius;

        if (kdtree.radiusSearch(searchPoint, radius,
                                pointIdxRadius, pointsSquaredDistRadius) > 0)
        {
            for (size_t j = 0; j < pointIdxRadius.size(); ++j)
            {
                float distance;
                distance = sqrtf((searchPoint.x - middle_Cloud_pointer->points[pointIdxRadius[j]].x) * (searchPoint.x - middle_Cloud_pointer->points[pointIdxRadius[j]].x) + (searchPoint.y - middle_Cloud_pointer->points[pointIdxRadius[j]].y) * (searchPoint.y - middle_Cloud_pointer->points[pointIdxRadius[j]].y) + (searchPoint.z - middle_Cloud_pointer->points[pointIdxRadius[j]].z) * (searchPoint.z - middle_Cloud_pointer->points[pointIdxRadius[j]].z));

                if ((middle_Cloud_pointer->points[pointIdxRadius[j]].z - middle_Cloud_pointer->points[pointIdxRadius[j]].x) > (searchPoint.z - searchPoint.x))
                {
                    //upper=true;
                    if ((middle_Cloud_pointer->points[pointIdxRadius[j]].z + middle_Cloud_pointer->points[pointIdxRadius[j]].x) > (searchPoint.z + searchPoint.x))
                    {
                        //inupper=true;
                        if (distance > mradius)
                        {
                            farup = true;
                        }
                        else
                        {
                            up = true;
                        }
                    }
                    else
                    {
                        if (distance > mradius)
                        {
                            farfront = true;
                        }
                        else
                        {
                            front = true;
                        }
                    }
                }
                else if ((middle_Cloud_pointer->points[pointIdxRadius[j]].z - middle_Cloud_pointer->points[pointIdxRadius[j]].x) < (searchPoint.z - searchPoint.x))
                {
                    if ((middle_Cloud_pointer->points[pointIdxRadius[j]].z + middle_Cloud_pointer->points[pointIdxRadius[j]].x) > (searchPoint.z + searchPoint.x))
                    {
                        //inupper=true;
                        if (distance > mradius)
                        {
                            farback = true;
                        }
                        else
                        {
                            back = true;
                        }
                    }
                    else
                    {
                        if (distance > mradius)
                        {
                            fardown = true;
                        }
                        else
                        {
                            down = true;
                        }
                    }
                }

                if ((up == true && down == true) || (front == true && back == true))
                {
                    break;
                }
            }
            if (up == true && farup == true)
            {
                rup = true;
            }
            if (down == true && fardown == true)
            {
                rdown = true;
            }
            if (front == true && farfront == true)
            {
                rfront = true;
            }
            if (back == true && farback == true)
            {
                rback = true;
            }

            if (rup == true && rfront == true && rback == false && rdown == false)
            { //concave
                //searchPoint.r=250;
                //searchPoint.b=0;
                pcl::PointXYZRGB color_point;
                color_point.x = searchPoint.x;
                color_point.y = searchPoint.y;
                color_point.z = searchPoint.z;
                color_point.r = 250;
                middle_Color_pointer->push_back(color_point);
                //temp_Cloud.push_back(searchPoint);
                Con concavepoint;
                concavepoint.point = searchPoint;
                concavepoint.count = 0;
                concave_set.push_back(concavepoint);

                concave.push_back(searchPoint);
            }
            else if (rup == false && rfront == false && rback == true && rdown == true)
            { //convex
                //searchPoint.g=250;
                //searchPoint.b=0;
                pcl::PointXYZRGB color_point;
                color_point.x = searchPoint.x;
                color_point.y = searchPoint.y;
                color_point.z = searchPoint.z;
                color_point.g = 250;
                middle_Color_pointer->push_back(color_point);
                //temp_Cloud.push_back(searchPoint);
                Con convexpoint;
                convexpoint.point = searchPoint;
                convexpoint.count = 0;
                convex_set.push_back(convexpoint);

                convex.push_back(searchPoint);
            }
        }
    }

    //matching process
    if (concave_set.size() == 0)
    {
        newconcave_set.clear();
    }
    else
    {
        newconcave_set = concave_set;
    }

    if (convex_set.size() == 0)
    {
        newconvex_set.clear();
    }
    else
    {
        newconvex_set = convex_set;
    }

    vector<Con> tempconcave_set;
    vector<Con> tempconvex_set;
    Con a;
    tempconcave_set.push_back(a);
    tempconvex_set.push_back(a);
    tempconcave_set.clear();
    tempconvex_set.clear();

    cout << oldconcave_set.size() << endl;

    if (oldconcave_set.size() == 0)
    {
        oldconcave_set = newconcave_set;
    }
    if (oldconvex_set.size() == 0)
    {
        oldconvex_set = newconvex_set;
    }

    //concave
    if (oldconcave_set.size() > 0)
    {
        for (size_t k = 0; k < newconcave_set.size(); ++k)
        {
            newconcave_set[k].pri_existance=false;
        }
        // cout<<oldconcave_set.size()<<endl;
        for (size_t i = 0; i < oldconcave_set.size(); ++i)
        {
            oldconcave_set[i].count += 1;
            for (size_t j = 0; j < newconcave_set.size(); ++j)
            {
                if(newconcave_set[j].pri_existance == true)
                {
                    newconcave_set[j].pri_existance =true;
                }
                float norm;
                float difference_x;
                float difference_z;
                difference_x = oldconcave_set[i].point.x - newconcave_set[j].point.x;
                difference_z = oldconcave_set[i].point.z - newconcave_set[j].point.z;
                norm = sqrtf((difference_x * difference_x) + (difference_z * difference_z));
                if (norm < 0.05)
                {
                    newconcave_set[j].count = 0;
                    newconcave_set[j].x_diff = difference_x;
                    newconcave_set[j].z_diff = difference_z;
                    newconcave_set[j].same_index = int(i);
                    newconcave_set[j].pri_existance = true;

                    tempconcave_set.push_back(newconcave_set[j]);
                }
            }
        }

        std::map<int, int> counting_instance_concave;
        for (vector<Con>::iterator it = tempconcave_set.begin(), it_end = tempconcave_set.end(); it != it_end; it++)
        {
            int instance = it->same_index;
            if (!counting_instance_concave.count(instance))
                counting_instance_concave[instance] = 0;
            counting_instance_concave[instance]++;
        }
        std::vector<int> uniques_concave;

        for (std::map<int, int>::iterator it = counting_instance_concave.begin(), it_end = counting_instance_concave.end(); it != it_end; it++)
        {
            int instance = it->first;
            int count = it->second;

            if (count == 1)
            {
                uniques_concave.push_back(instance);
            }
        }

        std::vector<Con> unique_set_concave;
        for (size_t j = 0; j < uniques_concave.size(); ++j)
        {

            for (size_t i = 0; i < tempconcave_set.size(); ++i)
            {
                if (tempconcave_set[i].same_index == uniques_concave[j])
                {
                    unique_set_concave.push_back(tempconcave_set[i]);
                }
            }
        }

        if (unique_set_concave.size() != 0)
        {
            tempconcave_set = unique_set_concave;
        }
    }

    //convex
    if (oldconvex_set.size() > 0)
    {
        for (size_t k = 0; k < newconvex_set.size(); ++k)
        {
            newconvex_set[k].pri_existance=false;
        }
        for (size_t i = 0; i < oldconvex_set.size(); ++i)
        {
            oldconvex_set[i].count++;
            for (size_t j = 0; j < newconvex_set.size(); ++j)
            {
                if(newconvex_set[j].pri_existance == true)
                {
                    newconvex_set[j].pri_existance =true;
                }
                float norm;
                float difference_x;
                float difference_z;
                difference_x = oldconvex_set[i].point.x - newconvex_set[j].point.x;
                difference_z = oldconvex_set[i].point.z - newconvex_set[j].point.z;
                norm = sqrtf((difference_x * difference_x) + (difference_z * difference_z));
                if (norm < 0.05)
                {
                    newconvex_set[j].count = 0;
                    newconvex_set[j].x_diff = difference_x;
                    newconvex_set[j].z_diff = difference_z;
                    newconvex_set[j].same_index = int(i);
                    newconvex_set[j].pri_existance = true;

                    tempconvex_set.push_back(newconvex_set[j]);
                }
            }
        }

        std::map<int, int> counting_instance_convex;
        for (vector<Con>::iterator it = tempconvex_set.begin(), it_end = tempconvex_set.end(); it != it_end; it++)
        {
            int instance = it->same_index;
            if (!counting_instance_convex.count(instance))
                counting_instance_convex[instance] = 0;
            counting_instance_convex[instance]++;
        }

        std::vector<int> uniques_convex;
        for (std::map<int, int>::iterator it = counting_instance_convex.begin(), it_end = counting_instance_convex.end(); it != it_end; it++)
        {
            int instance = it->first;
            int count = it->second;

            if (count == 1)
            {
                uniques_convex.push_back(instance);
            }
        }

        std::vector<Con> unique_set_convex;
        for (size_t j = 0; j < uniques_convex.size(); ++j)
        {

            for (size_t i = 0; i < tempconvex_set.size(); ++i)
            {
                if (tempconvex_set[i].same_index == uniques_convex[j])
                {
                    unique_set_convex.push_back(tempconvex_set[i]);
                }
            }
        }

        if (unique_set_convex.size() != 0)
        {
            tempconvex_set = unique_set_convex;
        }
    }

    //     //////////////////////////////
    cout << "old 1 concave size " << oldconcave_set.size();
    if (oldconcave_set.size() > 0)
    {
        for (size_t i = 0; i < oldconcave_set.size(); ++i)
        {
            cout << "[" << oldconcave_set[i].point.x << "," << oldconcave_set[i].point.z << "," << oldconcave_set[i].count << "] ";
        }
        cout << endl;
    }
    else
    {
        cout << endl;
    }

    cout << "new 1 concave size " << newconcave_set.size();
    if (newconcave_set.size() > 0)
    {
        for (size_t i = 0; i < newconcave_set.size(); ++i)
        {
            cout << "[" << newconcave_set[i].point.x << "," << newconcave_set[i].point.z << "," << newconcave_set[i].count << "] ";
        }
        cout << endl;
    }
    else
    {
        cout << endl;
    }
    cout << "temp 1 concave size " << tempconcave_set.size();
    if (tempconcave_set.size() > 0)
    {
        for (size_t i = 0; i < tempconcave_set.size(); ++i)
        {
            cout << "[" << tempconcave_set[i].point.x << "," << tempconcave_set[i].point.z << "," << tempconcave_set[i].count << "] ";
        }
        cout << endl;
    }
    else
    {
        cout << endl;
    }
    cout << "old 1 convex size " << oldconvex_set.size();
    if (oldconvex_set.size() > 0)
    {
        for (size_t i = 0; i < oldconvex_set.size(); ++i)
        {
            cout << "[" << oldconvex_set[i].point.x << "," << oldconvex_set[i].point.z << "," << oldconvex_set[i].count << "] ";
        }
        cout << endl;
    }
    else
    {
        cout << endl;
    }

    cout << "new 1 convex size " << newconvex_set.size();
    if (newconvex_set.size() > 0)
    {
        for (size_t i = 0; i < newconvex_set.size(); ++i)
        {
            cout << "[" << newconvex_set[i].point.x << "," << newconvex_set[i].point.z << "," << newconvex_set[i].count << "] ";
        }
        cout << endl;
    }
    else
    {
        cout << endl;
    }
    cout << "temp 1 convex size " << tempconvex_set.size();
    if (tempconvex_set.size() > 0)
    {
        for (size_t i = 0; i < tempconvex_set.size(); ++i)
        {
            cout << "[" << tempconvex_set[i].point.x << "," << tempconvex_set[i].point.z << "," << tempconvex_set[i].count << "] ";
        }
        cout << endl;
    }
    else
    {
        cout << endl;
    }
    cout << endl;
    ///////////////////////////

    //compensation
    float x_diff_sum=0.0;
    float z_diff_sum=0.0;
    float x_diff_avg=0.0;
    float z_diff_avg=0.0;

    if (tempconcave_set.size() == 0)
    {
        tempconcave_set.clear();
    }
    if (tempconvex_set.size() == 0)
    {
        tempconvex_set.clear();
    }

    for (size_t i = 0; i < tempconcave_set.size(); ++i)
    {
        x_diff_sum += tempconcave_set[i].x_diff;
        z_diff_sum += tempconcave_set[i].z_diff;
        cout << "cave x_diff " << tempconcave_set[i].x_diff << endl;
        cout << "cave z_diff " << tempconcave_set[i].z_diff << endl;
    }
    for (size_t i = 0; i < tempconvex_set.size(); ++i)
    {
        x_diff_sum += tempconvex_set[i].x_diff;
        z_diff_sum += tempconvex_set[i].z_diff;
        cout << "vex x_diff " << tempconvex_set[i].x_diff << endl;
        cout << "vex z_diff " << tempconvex_set[i].z_diff << endl;
    }
    cout<<"size + size "<<float(tempconcave_set.size() + tempconvex_set.size())<<endl;

    if (tempconcave_set.size() + tempconvex_set.size() != 0)
    {
        x_diff_avg = x_diff_sum / float(tempconcave_set.size() + tempconvex_set.size());
        z_diff_avg = z_diff_sum / float(tempconcave_set.size() + tempconvex_set.size());
    }
    else if (tempconcave_set.size() + tempconvex_set.size() == 0)
    {
        x_diff_avg = 0.0;
        z_diff_avg = 0.0;
    }
    cout << "x_diff avg" << x_diff_avg << endl;
    cout << "z_diff avg" << z_diff_avg << endl;

    for (size_t i = 0; i < temp_Cloud.points.size(); ++i)
    {
        temp_Cloud.points[i].x = temp_Cloud.points[i].x + x_diff_avg;
        temp_Cloud.points[i].z = temp_Cloud.points[i].z + z_diff_avg;
    }
    alpha += x_diff_avg;
    gamma += z_diff_avg;

    //updating old set (concave)
    for (size_t i = 0; i < newconcave_set.size(); ++i)
    {
        newconcave_set[i].point.x += x_diff_avg;
        newconcave_set[i].point.z += z_diff_avg;
        if (newconcave_set[i].pri_existance == true)
        {
            // oldconcave_set[newconcave_set[i].same_index] = newconcave_set[i];
            oldconcave_set[newconcave_set[i].same_index].count=0;
        }
    }
    for (size_t i = 0; i < newconcave_set.size(); ++i)
    {
        if (newconcave_set[i].pri_existance == false)
        {
            oldconcave_set.push_back(newconcave_set[i]);
        }
    }

    //updating old set (convex)
    for (size_t i = 0; i < newconvex_set.size(); ++i)
    {
        newconvex_set[i].point.x += x_diff_avg;
        newconvex_set[i].point.z += z_diff_avg;
        if (newconvex_set[i].pri_existance == true)
        {
            // oldconvex_set[newconvex_set[i].same_index] = newconvex_set[i];
            oldconvex_set[newconvex_set[i].same_index].count=0;
        }
    }
    for (size_t i = 0; i < newconvex_set.size(); ++i)
    {
        if (newconvex_set[i].pri_existance == false)
        {
            oldconvex_set.push_back(newconvex_set[i]);
        }
    }

    //eliminate
    for (vector<Con>::iterator it = oldconcave_set.begin(); it != oldconcave_set.end();)
    {
        if (it->count == 3)
        {
            it = oldconcave_set.erase(it);
        }
        else
        {
            it++;
        }
    }

    for (vector<Con>::iterator it = oldconvex_set.begin(); it != oldconvex_set.end();)
    {
        if (it->count == 3)
        {
            it = oldconvex_set.erase(it);
        }
        else
        {
            it++;
        }
    }

    cout << "old concave size " << oldconcave_set.size();
    if (oldconcave_set.size() > 0)
    {
        for (size_t i = 0; i < oldconcave_set.size(); ++i)
        {
            cout << "[" << oldconcave_set[i].point.x << "," << oldconcave_set[i].point.z << "," << oldconcave_set[i].count << "] ";
        }
        cout << endl;
    }
    else
    {
        cout << endl;
    }

    cout << "new concave size " << newconcave_set.size();
    if (newconcave_set.size() > 0)
    {
        for (size_t i = 0; i < newconcave_set.size(); ++i)
        {
            cout << "[" << newconcave_set[i].point.x << "," << newconcave_set[i].point.z << "," << newconcave_set[i].count << "] ";
        }
        cout << endl;
    }
    else
    {
        cout << endl;
    }

    cout << "old convex size " << oldconvex_set.size();
    if (oldconvex_set.size() > 0)
    {
        for (size_t i = 0; i < oldconvex_set.size(); ++i)
        {
            cout << "[" << oldconvex_set[i].point.x << "," << oldconvex_set[i].point.z << "," << oldconvex_set[i].count << "] ";
        }
        cout << endl;
    }
    else
    {
        cout << endl;
    }

    cout << "new convex size " << newconvex_set.size();
    if (newconvex_set.size() > 0)
    {
        for (size_t i = 0; i < newconvex_set.size(); ++i)
        {
            cout << "[" << newconvex_set[i].point.x << "," << newconvex_set[i].point.z << "," << newconvex_set[i].count << "] ";
        }
        cout << endl;
    }
    else
    {
        cout << endl;
    }
    cout << endl;

    // Convert To ROS data type (mapped pcl)
    sensor_msgs::PointCloud2 mapping_pcl;
    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(temp_Cloud, cloud_p);

    pcl_conversions::fromPCL(cloud_p, mapping_pcl);
    mapping_pcl.header.frame_id = frame_id;
    pub.publish(mapping_pcl);

    // Convert To ROS data type (line)
    sensor_msgs::PointCloud2 line;
    pcl::PCLPointCloud2 cloud_p1;
    pcl::toPCLPointCloud2(*middle_Color_pointer, cloud_p1);

    pcl_conversions::fromPCL(cloud_p1, line);
    line.header.frame_id = frame_id;
    pub1.publish(line);

    std_msgs::Float64 msga;
    msga.data=alpha;
    pub2.publish(msga);

    std_msgs::Float64 msgg;
    msgg.data=gamma;
    pub3.publish(msgg); 
}
