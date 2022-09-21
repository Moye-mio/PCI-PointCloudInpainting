#include "pch.h"

class TestPM : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}

	Eigen::MatrixXf castCV2Eigen(const cv::Mat& vCvMat)
	{
		Eigen::MatrixXf Mat(vCvMat.rows, vCvMat.cols);
		for (int i = 0; i < Mat.rows(); i++)
			for (int k = 0; k < Mat.cols(); k++)
				Mat.coeffRef(i, k) = vCvMat.at<unsigned char>(i, k);
		return Mat;
	}
};

TEST_F(TestPM, NT_PMTest)
{
	cv::Mat Image = cv::imread("Images/test.png", cv::IMREAD_GRAYSCALE);
	cv::Mat Mask = cv::imread("Images/mask.png", cv::IMREAD_GRAYSCALE);

	cv::Mat Image2(Image.size(), CV_32F);
	for (int i = 0; i < Image.rows; i++)
		for (int k = 0; k < Image.cols; k++)
			Image2.at<float>(i, k) = (float)Image.at<unsigned char>(i, k);

	cv::Mat Result = PM::run(Image2, Mask);
	cv::imwrite("Images/Result.png", Result);
	cv::imshow("Result", Result);
	cv::waitKey();
}

//TEST_F(TestPM, NT_)
//{
//	cv::Mat Image = cv::imread("test.png", cv::IMREAD_GRAYSCALE);
//	cv::Mat Mask = cv::imread("mask.png", cv::IMREAD_GRAYSCALE);
//	Eigen::MatrixXf ImageMat = castCV2Eigen(Image);
//	Eigen::MatrixXf MaskMat = castCV2Eigen(Mask);
//
//	core::CHeightMap ImageMap, MaskMap;
//	ImageMap.setHeightMap(ImageMat);
//	MaskMap.setHeightMap(MaskMat);
//
//	RunPatchMatch(ImageMap, MaskMap);
//}