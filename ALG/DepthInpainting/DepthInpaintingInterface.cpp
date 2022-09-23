#include "pch.h"
#include "DepthInpaintingInterface.h"

using namespace depthInpainting;

void castMap2CV(const core::CGradientMap& vMap, cv::Mat& voMat, float vCoeff)
{
	voMat = cv::Mat(cv::Size(vMap.getWidth(), vMap.getHeight()), CV_32FC2);
	for (int i = 0; i < voMat.rows; i++)
		for (int k = 0; k < voMat.cols; k++)
			voMat.at<cv::Vec2f>(i, k) = cv::Vec2f(vMap.getValueAt(i, k)[0] * vCoeff, vMap.getValueAt(i, k)[1] * vCoeff);
}

void castMap2CV(const core::CHeightMap& vMap, cv::Mat& voMat)
{
	voMat = cv::Mat(cv::Size(vMap.getWidth(), vMap.getHeight()), CV_8UC1);
	for (int i = 0; i < voMat.rows; i++)
		for (int k = 0; k < voMat.cols; k++)
			voMat.at<unsigned char>(i, k) = (unsigned char)vMap.getValueAt(i, k);
}

void castCV2Map(const cv::Mat& vMat, core::CGradientMap& vMap)
{
	vMap.setSize(vMat.rows, vMat.cols);
	for (int i = 0; i < vMat.rows; i++)
		for (int k = 0; k < vMat.cols; k++)
			vMap.setValueAt(Eigen::Vector2f(vMat.at<cv::Vec2f>(i, k)[0], vMat.at<cv::Vec2f>(i, k)[1]), i, k);
}

void setUpEquations(const core::CHeightMap& vHeightMap, const core::CGradientMap& vGoG)
{

}

void depthInpainting::runDepthInpaiting(const PC_t::Ptr& vCloud, PC_t::Ptr& voResultCloud)
{
	_ASSERTE(vCloud != nullptr);
	_ASSERTE(vCloud->size());

	core::CHeightMap HeightMap;
	core::CHeightMapGenerator HGenerator;
	HGenerator.setCloud(vCloud);
	HGenerator.dumpHeightMap(HeightMap);
	_ASSERTE(HeightMap.isValid());

	core::CGradientMap GradientMap;
	core::CGradientMapGenerator GGenerator;
	GGenerator.generate(HeightMap, true);
	GGenerator.dumpGradientMap(GradientMap);
	_ASSERTE(GradientMap.isValid());

	core::CHeightMap MaskMap;
	GradientMap.generateMask(MaskMap);
	_ASSERTE(MaskMap.isValid());

	cv::Mat Raw, Mask;
	castMap2CV(GradientMap, Raw, 1000);
	castMap2CV(MaskMap, Mask);
	cv::Mat Result = PM::run(Raw, Mask);

	core::CGradientMap GradientMapFilled, GoG;
	castCV2Map(Result, GradientMapFilled);
	GGenerator.generate(GradientMapFilled);
	GGenerator.dumpGradientMap(GoG);



}

