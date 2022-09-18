#pragma once

#include <xstring>
#include <Eigen/Eigen>
#include "HeightMap.h"

int RunPatchMatch(const core::CHeightMap& vImage, const core::CHeightMap& vMask);