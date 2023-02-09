#include "pch.h"

#include "ImageInpainting.h"
#include "GradientMapGenerator.h"
#include "PMInterface.h"
#include "SolverBuilder.h"
#include "SparseLinearSolver.h"

using namespace dataManagement;

bool CImageInpainting::run(const core::CHeightMap& vMap, core::CHeightMap& voInpainted)
{
	_HIVE_EARLY_RETURN(vMap.isValid() == false, "Image Inpainting: Map is Invalid", false);

	{
		for (int i = 0; i < vMap.getWidth(); i++)
			for (int k = 0; k < vMap.getHeight(); k++)
				std::cout << "Map: [" << i << ", " << k << "], Value: " << vMap.getValueAt(i, k) << std::endl;
	}

	/* Create GradientMap */
	core::CGradientMap GradientMap;
	core::CGradientMapGenerator GGenerator;
	_HIVE_EARLY_RETURN(GGenerator.generate(vMap, true) == false, "Image Inpainting: Gradient Map Generate failed", false);
	GGenerator.dumpGradientMap(GradientMap);
	_HIVE_EARLY_RETURN(GradientMap.isValid() == false, "Image Inpainting: GradientMap is InValid", false);

	{
		for (int i = 0; i < GradientMap.getWidth(); i++)
			for (int k = 0; k < GradientMap.getHeight(); k++)
				std::cout << "Gradient: [" << i << ", " << k << "], Value: [" << GradientMap.getValueAt(i, k)[0] << ", " << GradientMap.getValueAt(i, k)[1] << "]" << std::endl;
	}

	/* Create Mask */
	core::CHeightMap MaskMap;
	GradientMap.generateMask(MaskMap);
	_HIVE_EARLY_RETURN(MaskMap.isValid() == false, "Image Inpainting: Mask is Invalid", false);

	/* PM */
	float GradientMapCoef = 1000.0f;
	float GradientFilledMapCoef = 0.001f;
	float MaskCoef = 1.0f;
	int PatchSize = (std::min(vMap.getWidth(), vMap.getHeight()) < 22) ? (std::min(vMap.getWidth(), vMap.getHeight()) / 2) : 11;
	cv::Mat Raw, Mask;
	__castMap2CV(GradientMap, Raw, GradientMapCoef);
	__castMap2CV(MaskMap, Mask, MaskCoef);

	{
		auto GMask = Mask;
		for (int i = 0; i < GMask.rows; i++)
			for (int k = 0; k < GMask.cols; k++)
				GMask.at<unsigned char>(i, k) *= 255;
			cv::imwrite("GradientMask.png", GMask);
	}
	cv::Mat Result = PM::run(Raw, Mask, PatchSize);

	core::CGradientMap GradientMapFilled, GoG;
	__castCV2Map(Result, GradientMapFilled, GradientFilledMapCoef);
	_HIVE_EARLY_RETURN(GradientMapFilled.isValid() == false, "Image Inpainting: PM Result is Invalid", false);
	_HIVE_EARLY_RETURN(GGenerator.generate(GradientMapFilled) == false, "Image Inpainting: GOG generate failed", false);
	GGenerator.dumpGradientMap(GoG);
	_HIVE_EARLY_RETURN(GoG.isValid() == false, "Image Inpainting: GoG is InValid", false);

	/* Set up Equations */
	Eigen::MatrixXf Coeff, ConstNumbers, Solutions;
	std::vector<Eigen::Vector2f> Unknowns;
	dataManagement::CSolverBuilder SolverBuilder;
	_HIVE_EARLY_RETURN(SolverBuilder.setUp(vMap, GoG, GradientMapFilled) == false, "Image Inpainting: Solver Builder set up failed", false);
	SolverBuilder.dumpMatrix(Coeff, ConstNumbers);
	SolverBuilder.dumpUnknowns(Unknowns);

	/* Solve Equations */
	core::CSparseLinearSolver Solver(Coeff, ConstNumbers);
	Solutions = Solver.solve(1);
	core::CHeightMap HeightMapInpainted;
	HeightMapInpainted = vMap;
	_HIVE_EARLY_RETURN(__setValueInMap(HeightMapInpainted, Unknowns, Solutions) == false, "Image Inpainting: Set Value failed", false);

	voInpainted = HeightMapInpainted;

	return true;
}

void CImageInpainting::__castMap2CV(const core::CHeightMap& vMap, cv::Mat& voMat, float vCoeff)
{
	voMat = cv::Mat(cv::Size(vMap.getWidth(), vMap.getHeight()), CV_8UC1);
	for (int i = 0; i < voMat.rows; i++)
		for (int k = 0; k < voMat.cols; k++)
			voMat.at<unsigned char>(i, k) = (unsigned char)vMap.getValueAt(i, k) * vCoeff;
}

void CImageInpainting::__castMap2CV(const core::CGradientMap& vMap, cv::Mat& voMat, float vCoeff)
{
	voMat = cv::Mat(cv::Size(vMap.getWidth(), vMap.getHeight()), CV_32FC2);
	for (int i = 0; i < voMat.rows; i++)
		for (int k = 0; k < voMat.cols; k++)
			voMat.at<cv::Vec2f>(i, k) = cv::Vec2f(vMap.getValueAt(i, k)[0] * vCoeff, vMap.getValueAt(i, k)[1] * vCoeff);
}

void CImageInpainting::__castCV2Map(const cv::Mat& vMat, core::CGradientMap& voMap, float vCoeff)
{
	voMap.setSize(vMat.rows, vMat.cols);
	for (int i = 0; i < vMat.rows; i++)
		for (int k = 0; k < vMat.cols; k++)
			voMap.setValueAt(Eigen::Vector2f(vMat.at<cv::Vec2f>(i, k)[0] * vCoeff, vMat.at<cv::Vec2f>(i, k)[1] * vCoeff), i, k);
}

bool CImageInpainting::__setValueInMap(core::CHeightMap& vioMap, const std::vector<Eigen::Vector2f>& vUnknowns, const Eigen::MatrixXf& vSolutions)
{
	_HIVE_EARLY_RETURN(vUnknowns.size() != (std::size_t)vSolutions.rows(), "Image Inpainting: vUnknowns size != Solutions rows", false);
	for (int i = 0; i < vUnknowns.size(); i++)
		vioMap.setValueAt(vSolutions(i, 0), vUnknowns[i][0], vUnknowns[i][1]);
	_HIVE_EARLY_RETURN(vioMap.isNoEmptyValue() == false, "Image Inpaiting: Result Map has empty value", false);
	return true;
}