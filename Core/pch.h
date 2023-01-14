﻿// pch.h: 这是预编译标头文件。
// 下方列出的文件仅编译一次，提高了将来生成的生成性能。
// 这还将影响 IntelliSense 性能，包括代码完成和许多代码浏览功能。
// 但是，如果此处列出的文件中的任何一个在生成之间有更新，它们全部都将被重新编译。
// 请勿在此处添加要频繁更新的文件，这将使得性能优势无效。

#ifndef PCH_H
#define PCH_H

// 添加要在此处预编译的标头
#include "framework.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <numbers>
#include <numeric>
#include <vector>
#include <string>
#include <map>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <optional>

#include <omp.h>
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

#include "PointCloudType.h"
#include "Interpolation.h"
#include "Plane.h"
#include "Point.h"
#include "Triangle.h"
#include "AABB.h"

#endif //PCH_H