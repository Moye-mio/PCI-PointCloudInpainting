// pch.h: 这是预编译标头文件。
//
// pch.h
//

#pragma once

#include "gtest/gtest.h"
#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <cstdlib>

#include <boost/format.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "common/Product.h"
#include "common/Factory.h"
#include "common/Singleton.h"
#include "common/CommonInterface.h"
#include "common/EventLoggerInterface.h"
#include "common/DesignPatternInterface.h"
#include "common/UtilityInterface.h"
#include "common/MathInterface.h"

#include "magic_enum.hpp"
#include "PointCloudType.h"
#include "HeightMap.h"
#include "GradientMap.h"
#include "AABB.h"
#include "AABBEstimation.h"
#include "DepthInpainting.h"
#include "HeightMapGenerator.h"
#include "GradientMapGenerator.h"
#include "SolverBuilder.h"
#include "SparseLinearSolver.h"
#include "HeightMap2PCMapper.h"
#include "PCLoader.h"
