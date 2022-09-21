#pragma once

#include <xstring>
#include <Eigen/Eigen>
#include "HeightMap.h"
#include "GradientMap.h"

void RunPatchMatch(const core::CGradientMap& vImage, const core::CHeightMap& vMask);

void RunPatchMatch(const core::CHeightMap& vImage, const core::CHeightMap& vMask);