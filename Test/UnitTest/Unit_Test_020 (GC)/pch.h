//
// pch.h
//

#pragma once

#include "gtest/gtest.h"

#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <cstdlib>
#include <numbers>
#include <array>

#include <boost/format.hpp>

#include <pcl/io/ply_io.h>
#include <vtkRenderWindow.h>

#include "common/Product.h"
#include "common/Factory.h"
#include "common/Singleton.h"
#include "common/CommonInterface.h"
#include "common/EventLoggerInterface.h"
#include "common/DesignPatternInterface.h"
#include "common/UtilityInterface.h"
#include "common/MathInterface.h"
#include "common/CpuTimer.h"

#include "PointCloudType.h"

#include "GeneticClustering.h"
#include "Cluster.h"
#include "NormalEstimator.h"