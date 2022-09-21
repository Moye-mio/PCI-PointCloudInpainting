#include "pch.h"
#include "PMInterface.h"
#include "Inpaint.h"

using namespace PM;

/*
 Mask: 0 raw; 1 to be filled
*/
cv::Mat PM::run(const cv::Mat& vRaw, const cv::Mat& vMask, int vPatchSize /*= 11*/)
{
	_ASSERTE(vPatchSize > 0);
	_ASSERTE(vRaw.data && vMask.data);
	_ASSERTE(vRaw.rows == vMask.rows && vRaw.cols == vMask.cols);

	cv::VideoWriter VideoWriter("PM.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25.0, cv::Size(vRaw.cols, vRaw.rows));

	CInpainter Inpainter(vRaw, vMask, vPatchSize);
	Inpainter.inpaint(VideoWriter);
	cv::Mat Result;
	Inpainter.dumpResult(Result);
	VideoWriter.release();

	return Result;
}
