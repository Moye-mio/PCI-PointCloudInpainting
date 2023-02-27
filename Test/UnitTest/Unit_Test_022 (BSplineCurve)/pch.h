//
// pch.h
//

#pragma once

#include "gtest/gtest.h"

#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include <numbers>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

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

#include "Point.h"
#include "BSplineCurve.h"
