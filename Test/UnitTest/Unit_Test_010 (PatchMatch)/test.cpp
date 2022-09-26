#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/SlantPyramid.ply");

class TestPM : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}
};

//TEST_F(TestPM, NT_FloatPM)
//{
//	cv::Mat Image = cv::imread("Images/Pyramid.png", cv::IMREAD_GRAYSCALE);
//	cv::Mat Mask = cv::imread("Images/Pyramid-mask.png", cv::IMREAD_GRAYSCALE);
//
//	cv::Mat Image2(Image.size(), CV_32F);
//	for (int i = 0; i < Image.rows; i++)
//		for (int k = 0; k < Image.cols; k++)
//			Image2.at<float>(i, k) = (float)Image.at<unsigned char>(i, k);
//
//	cv::Mat Result = PM::run(Image2, Mask);
//	cv::imwrite("Images/Result.png", Result);
//	cv::imshow("Result", Result);
//	cv::waitKey();
//}

TEST_F(TestPM, NT_Channel2)
{
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<dataManagement::IPCLoader>(hiveUtility::hiveGetFileSuffix(ModelPath));
	ASSERT_TRUE(pTileLoader);
	PC_t::Ptr pData = pTileLoader->loadDataFromFile(ModelPath);
	ASSERT_TRUE(pData);

	core::CHeightMap HeightMap;
	core::CHeightMapGenerator HMGenerator;
	HMGenerator.setCloud(pData);
	HMGenerator.generate(128, 128);
	HMGenerator.dumpHeightMap(HeightMap);
	_ASSERTE(HeightMap.isValid());

	core::CGradientMap GradientMap;
	core::CGradientMapGenerator GMGenerator;
	GMGenerator.generate(HeightMap);
	GMGenerator.dumpGradientMap(GradientMap);
	_ASSERTE(GradientMap.isValid());

	cv::Mat Raw(cv::Size(GradientMap.getWidth(), GradientMap.getHeight()), CV_32FC2);
	for (int i = 0; i < Raw.rows; i++)
		for (int k = 0; k < Raw.cols; k++)
		{
			if (GradientMap.isEmptyValue(i, k))
			{
				Raw.at<cv::Vec2f>(i, k)[0] = -FLT_MAX;
				Raw.at<cv::Vec2f>(i, k)[1] = -FLT_MAX;
			}
			else
			{
				Raw.at<cv::Vec2f>(i, k)[0] = GradientMap.getValueAt(i, k)[0] * 1000;
				Raw.at<cv::Vec2f>(i, k)[1] = GradientMap.getValueAt(i, k)[1] * 1000;
			}
		}

	core::CHeightMap MaskMap;
	GradientMap.generateMask(MaskMap);
	_ASSERTE(MaskMap.isValid());

	cv::Mat Mask(cv::Size(GradientMap.getWidth(), GradientMap.getHeight()), CV_8UC1);
	for (int i = 0; i < Mask.rows; i++)
		for (int k = 0; k < Mask.cols; k++)
			Mask.at<unsigned char>(i, k) = MaskMap.getValueAt(i, k) * 255;

	cv::Mat Result = PM::run(Raw, Mask);

	cv::Mat ResultC1(Result.size(), CV_8UC1), ResultC2(Result.size(), CV_8UC1);
	for (int i = 0; i < Result.rows; i++)
		for (int k = 0; k < Result.cols; k++)
		{
			ResultC1.at<unsigned char>(i, k) = (unsigned char)(Result.at<cv::Vec2f>(i, k)[0]);
			ResultC2.at<unsigned char>(i, k) = (unsigned char)(Result.at<cv::Vec2f>(i, k)[1]);
		}

	cv::imwrite("CX.png", ResultC1);
	cv::imwrite("CY.png", ResultC2);
}