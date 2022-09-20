#include "run.h"
#include "inpainter.h"

int RunPatchMatch(const core::CHeightMap& vImage, const core::CHeightMap& vMask)
{	
	int HalfPatchWidth = 5;

	cv::Mat OriginalImage(vImage.getWidth(), vImage.getHeight(), CV_8UC3);
	for (int i = 0; i < vImage.getWidth(); i++)
		for (int k = 0; k < vImage.getHeight(); k++)
		{
			OriginalImage.at<cv::Vec3b>(i, k)[0] = vImage.getValueAt(i, k);
			OriginalImage.at<cv::Vec3b>(i, k)[1] = vImage.getValueAt(i, k);
			OriginalImage.at<cv::Vec3b>(i, k)[2] = vImage.getValueAt(i, k);
		}

	cv::Mat InpaintMask(vMask.getWidth(), vMask.getHeight(), CV_8UC1);
	for (int i = 0; i < vMask.getWidth(); i++)
		for (int k = 0; k < vMask.getHeight(); k++)
			InpaintMask.at<unsigned char>(i, k) = vMask.getValueAt(i, k);


	_ASSERTE(OriginalImage.data);
	_ASSERTE(InpaintMask.data);

	cv::Mat Image = OriginalImage.clone();
	bool MaskSpecified = true;
	
	cv::VideoWriter VideoWrite(".\\test.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25.0, cv::Size(OriginalImage.cols, OriginalImage.rows));

	Inpainter Inpainter(OriginalImage, InpaintMask, HalfPatchWidth);

	if (Inpainter.checkValidInputs() == Inpainter.CHECK_VALID) {
		Inpainter.inpaint(VideoWrite);

		VideoWrite.release();

		cv::imwrite("1-result.png", Inpainter.m_Result);
		cv::namedWindow("1-result");
		cv::imshow("1-result", Inpainter.m_Result);
		cv::waitKey();
	}
	else {
		std::cout << std::endl << "Error : invalid parameters" << std::endl;
	}

	return 0;
}