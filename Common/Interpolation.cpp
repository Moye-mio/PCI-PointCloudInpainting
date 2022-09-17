#include "Interpolation.h"

using namespace common;

float common::bilinearInterpolate(const std::pair<float, float>& vWeights, const std::vector<float>& vValues)
{
	_ASSERTE(vValues.size() >= 4);
	return (1 - vWeights.first) * (1 - vWeights.second) * vValues[0] + (1 - vWeights.first) * vWeights.second * vValues[1] + vWeights.first * (1 - vWeights.second) * vValues[2] + vWeights.first * vWeights.second * vValues[3];
}
