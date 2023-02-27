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
#include <optional>

#include <omp.h>
#include <boost/format.hpp>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include "common/Product.h"
#include "common/Factory.h"
#include "common/Singleton.h"
#include "common/CommonInterface.h"
#include "common/EventLoggerInterface.h"
#include "common/DesignPatternInterface.h"
#include "common/UtilityInterface.h"
#include "common/MathInterface.h"

#include "PointCloudType.h"
#include "AABB.h"
#include "HeightMap.h"
#include "Point.h"
#include "Triangle.h"
#include "Plane.h"
#include "Vertex.h"

#include "SimilarityEstimator.h"
#include "Geometric_distortion.h"
