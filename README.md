# Stairway_Mapping
Plane-Based Stairway Mapping Algorithm

## Information

**Author: Seungjun Woo<br />
Affiliation: [RISE](https://rise.skku.edu/)<br />
Maintainer: Seungjun Woo, wsj6928@gmail.com<br />**

![Screenshot from 2019-08-25 14-41-04](https://user-images.githubusercontent.com/35325906/71704356-379a2680-2e1d-11ea-8f55-26b7a0a43017.png)

## Installation

### Dependencies

This software is built on the Robotic Operating System ([ROS]), which needs to be [installed](http://wiki.ros.org) first. Additionally, it depends on following software:

- [Point Cloud Library (PCL)](http://pointclouds.org/) (point cloud processing).

### Building

In order to install this package, clone the latest version from this repository into your catkin workspace and compile the package using ROS.

    cd catkin_workspace/src
    git clone https://github.com/wsj6928/Stairway_Mapping.git
    cd ../
    catkin_make

## Parameter

![image](https://user-images.githubusercontent.com/35325906/71708165-8bfbd100-2e32-11ea-842a-6e5b79378cb0.png)

### Display
    
- **`disp_point_number`**         

display the number of points true/false

### Topic name

- **`map_publish_topic`**         

It refers to the name of the point-cloud topic that represents the segmented planes in the current moment.

- **`pcl_subscribe_topic`**       

It refers to the name of the input point-cloud topic that is transformed from **rosodom2** node.
 
- **`horizontal_publish_topic`**  

It refers to the name of the output point-cloud topic of the stairway mapping algorithm.

### Frame setting

- **`frame_id`**       

It refers to the name of the global frame.


### Pass filter

- **`fieldname`**                 

camera direction x,y, or z

- **`passlimit`**

pass limit in meter

### Voxel filter

- **`voxel`**      

voxel filtering grid size in meter

### Plane extraction

- **`MinClusterSize`** 

A minimum number of required points to be segmented as a plane.

- **`DistanceThreshold`**         

RANSAC sampling threshold in meter

- **`maxiter`**                   

The number of maximum iteration in RANSAC process

### stairstep

- **`maxangle`**                  

maximum angle from the ground in degree

- **`max_rel_angle`**             

Maximum allowed relative degree between each stairstep in a stairway 


