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
#include "AABB.h"
#include "HeightMap.h"
#include "Point.h"
#include "Triangle.h"
#include "Plane.h"
#include "Vertex.h"

#include "BSplineSurface.h"
#include "BSplineCurve.h"
#include "HeightMapGenerator.h"
#include "Surface2CloudMapper.h"
#include "SurfaceUVGenerator.h"
#include "VertexSampler.h"
#include "NormalSampler.h"
#include "DistSampler.h"
#include "Surface2PCMapper.h"
#include "MultilayerSurface.h"
#include "Image.h"

#include "PCLoader.h"
#include "SurfaceGenerator.h"