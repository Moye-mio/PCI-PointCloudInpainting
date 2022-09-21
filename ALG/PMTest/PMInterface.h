#pragma once

#include <opencv2/opencv.hpp>

namespace PM
{
	cv::Mat run(const cv::Mat& vRaw, const cv::Mat& vMask, int vPatchSize = 11);
}