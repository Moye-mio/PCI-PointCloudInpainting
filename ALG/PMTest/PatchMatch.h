#pragma once

namespace PM
{
	cv::Mat PatchMatch(const cv::Mat& vSource, const cv::Mat& vTarget, const cv::Mat& vMask, int vPatchSize);
}