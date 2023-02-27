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

#include "common/Product.h"
#include "common/Factory.h"
#include "common/Singleton.h"
#include "common/CommonInterface.h"
#include "common/EventLoggerInterface.h"
#include "common/DesignPatternInterface.h"
#include "common/UtilityInterface.h"
#include "common/MathInterface.h"
#include "common/CpuTimer.h"

#include "Vertex.h"
#include "Plane.h"
#include "Point.h"
#include "Triangle.h"
#include "HeightMap.h"
#include "SurfaceUVGenerator.h"