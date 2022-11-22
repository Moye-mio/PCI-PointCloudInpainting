#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using Point_t = pcl::PointXYZRGBA;
using RawPoint_t = pcl::PointXYZRGBNormal;
using Normal_t = pcl::PointNormal;

using PC_t = pcl::PointCloud<Point_t>;
using RawPC_t = pcl::PointCloud<RawPoint_t>;
using NormalPC_t = pcl::PointCloud<Normal_t>;
