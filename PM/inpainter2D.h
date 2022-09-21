#pragma once

#include <opencv2/opencv.hpp>

class Inpainter2D
{
public:
	const static int DEFAULT_HALF_PATCH_WIDTH = 3;
	const static int MODE_ADDITION = 0;
	const static int MODE_MULTIPLICATION = 1;
	const static int ERROR_INPUT_MAT_INVALID_TYPE = 0;
	const static int ERROR_INPUT_MASK_INVALID_TYPE = 1;
	const static int ERROR_MASK_INPUT_SIZE_MISMATCH = 2;
	const static int ERROR_HALF_PATCH_WIDTH_ZERO = 3;
	const static int CHECK_VALID = 4;

	Inpainter2D(const std::pair<cv::Mat, cv::Mat>& vImages, const cv::Mat& mask, int halfPatchWidth = 4, int mode = 1);

	void inpaint();

private:
	cv::Mat m_X;
	cv::Mat m_Y;
	cv::Mat m_Mask;
	cv::Mat m_Result;
	cv::Mat m_WorkX;
	cv::Mat m_WorkY;
	cv::Mat m_SourceRegion;
	cv::Mat m_TargetRegion;

	int m_HalfPatchWidth;
};