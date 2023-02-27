//
// pch.h
//

#pragma once

#include "gtest/gtest.h"

#include <vector>
#include <string>
#include <iostream>
#include <optional>

#include <boost/format.hpp>
#include <Eigen/Eigen>

#include <pcl/point_types.h>

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

#include "Point.h"
#include "ControlPointGenerator.h"
