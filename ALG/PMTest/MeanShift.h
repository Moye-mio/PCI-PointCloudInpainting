#pragma once

namespace PM
{
	float MeanShift(const std::vector<float>& vColors, const std::vector<float>& vWeights, int vSigma);
	cv::Vec2f MeanShift(const std::vector<cv::Vec2f>& vColors, const std::vector<float>& vWeights, int vSigma);
}