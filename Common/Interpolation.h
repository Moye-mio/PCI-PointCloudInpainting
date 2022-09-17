#pragma once

#include <utility>
#include <vector>

namespace common
{
	float bilinearInterpolate(const std::pair<float, float>& vWeights, const std::vector<float>& vValues);
}
