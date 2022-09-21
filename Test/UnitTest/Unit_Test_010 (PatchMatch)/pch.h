//
// pch.h
//

#pragma once

#include "gtest/gtest.h"

#include <vector>
#include <string>
#include <iostream>

#include <boost/format.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

#include "common/Product.h"
#include "common/Factory.h"
#include "common/Singleton.h"
#include "common/CommonInterface.h"
#include "common/EventLoggerInterface.h"
#include "common/DesignPatternInterface.h"
#include "common/UtilityInterface.h"

#include "PointCloudType.h"
#include "HeightMap.h"
#include "HeightMapGenerator.h"
#include "GradientMap.h"
#include "GradientMapGenerator.h"
#include "PMInterface.h"
#include "PCLoader.h"
#include "AABB.h"
#include "AABBEstimation.h"