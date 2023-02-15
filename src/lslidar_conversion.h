
#ifndef __LSLIDAR_CONVERSION_H
#define __LSLIDAR_CONVERSION_H


#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

using namespace std;

struct myPointXYZIR{			//~ Ring, Intensity, Distance(range)
	PCL_ADD_POINT4D; // quad-word XYZ
	float intensity; ///< laser intensity reading
	uint16_t ring; ///< laser ring number
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
};
POINT_CLOUD_REGISTER_POINT_STRUCT(
		myPointXYZIR, (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity) (uint16_t, ring, ring))

#endif

