#include "pch.h"
#include "DepthInpainting.h"
#include "magic_enum.hpp"
#include "HeightMapGenerator.h"
#include "GradientMapGenerator.h"
#include "PMInterface.h"
#include "SolverBuilder.h"
#include "SparseLinearSolver.h"
#include "HeightMap2PCMapper.h"

using namespace dataManagement;

void dataManagement::CDepthInpaiting::run(const PC_t::Ptr& vCloud, PC_t::Ptr& voResultCloud)
{
	_ASSERTE(vCloud != nullptr);
	_ASSERTE(vCloud->size());

	/* Create HeightMap */
	core::CHeightMap HeightMap, HeightMapInpainted;
	core::CHeightMapGenerator HGenerator;
	HGenerator.setCloud(vCloud);
	HGenerator.generate(128, 128);			/* Magic Number */
	HGenerator.dumpHeightMap(HeightMap);
	_ASSERTE(HeightMap.isValid());

	/* Create GradientMap */
	core::CGradientMap GradientMap;
	core::CGradientMapGenerator GGenerator;
	GGenerator.generate(HeightMap, true);
	GGenerator.dumpGradientMap(GradientMap);
	_ASSERTE(GradientMap.isValid());

	/* Create Mask */
	core::CHeightMap MaskMap;
	GradientMap.generateMask(MaskMap);
	_ASSERTE(MaskMap.isValid());

	/* PM */
	cv::Mat Raw, Mask;
	__castMap2CV(GradientMap, Raw, 1000);
	__castMap2CV(MaskMap, Mask, 1);
	cv::Mat Result = PM::run(Raw, Mask);

	core::CGradientMap GradientMapFilled, GoG;
	__castCV2Map(Result, GradientMapFilled, 0.001f);
	GGenerator.generate(GradientMapFilled);
	GGenerator.dumpGradientMap(GoG);

	/* Set up Equations */
	Eigen::MatrixXf Coeff, ConstNumbers, Solutions;
	std::vector<Eigen::Vector2f> Unknowns;
	dataManagement::CSolverBuilder SolverBuilder;
	SolverBuilder.setUp(HeightMap, GoG, GradientMapFilled);
	SolverBuilder.dumpMatrix(Coeff, ConstNumbers);
	SolverBuilder.dumpUnknowns(Unknowns);

	/* Solve Equations */
	core::CSparseLinearSolver Solver(Coeff, ConstNumbers);
	Solutions = Solver.solve(1);
	HeightMapInpainted = HeightMap;
	_ASSERTE(HeightMapInpainted.isValid());
	__setValueInMap(HeightMapInpainted, Unknowns, Solutions);

	{
		cv::Mat Save(cv::Size(HeightMapInpainted.getWidth(), HeightMapInpainted.getHeight()), CV_8UC1);
		for (int i = 0; i < Save.rows; i++)
			for (int k = 0; k < Save.cols; k++)
				Save.at<unsigned char>(i, k) = (unsigned char)(HeightMapInpainted.getValueAt(i, k) * 51);
		cv::imwrite("Save.png", Save);
		for (int i = 0; i < Mask.rows; i++)
			for (int k = 0; k < Mask.cols; k++)
				Mask.at<unsigned char>(i, k) *= 255;
		cv::imwrite("Mask.png", Mask);
	}

	/* Map to Cloud */
	core::CAABBEstimation Estimation(vCloud);
	core::SAABB Box = Estimation.compute();
	_ASSERTE(Box.isValid());
	core::CHeightMap2PCMapper Mapper;
	Mapper.map2PC(voResultCloud, std::make_pair(HeightMap, HeightMapInpainted), Box, 20);	/* Magic Number */
}

void CDepthInpaiting::__setValueInMap(core::CHeightMap& vioMap, const std::vector<Eigen::Vector2f>& vUnknowns, const Eigen::MatrixXf& vSolutions)
{
	_ASSERTE(vUnknowns.size() == (std::size_t)vSolutions.rows());
	for (int i = 0; i < vUnknowns.size(); i++)
		vioMap.setValueAt(vSolutions(i, 0), vUnknowns[i][0], vUnknowns[i][1]);
	_ASSERTE(vioMap.isNoEmptyValue());
}

void CDepthInpaiting::__castCV2Map(const cv::Mat& vMat, core::CGradientMap& voMap, float vCoeff)
{
	voMap.setSize(vMat.rows, vMat.cols);
	for (int i = 0; i < vMat.rows; i++)
		for (int k = 0; k < vMat.cols; k++)
			voMap.setValueAt(Eigen::Vector2f(vMat.at<cv::Vec2f>(i, k)[0] * vCoeff, vMat.at<cv::Vec2f>(i, k)[1] * vCoeff), i, k);
}

void CDepthInpaiting::__castMap2CV(const core::CHeightMap& vMap, cv::Mat& voMat, float vCoeff)
{
	voMat = cv::Mat(cv::Size(vMap.getWidth(), vMap.getHeight()), CV_8UC1);
	for (int i = 0; i < voMat.rows; i++)
		for (int k = 0; k < voMat.cols; k++)
			voMat.at<unsigned char>(i, k) = (unsigned char)vMap.getValueAt(i, k) * vCoeff;
}

void CDepthInpaiting::__castMap2CV(const core::CGradientMap& vMap, cv::Mat& voMat, float vCoeff)
{
	voMat = cv::Mat(cv::Size(vMap.getWidth(), vMap.getHeight()), CV_32FC2);
	for (int i = 0; i < voMat.rows; i++)
		for (int k = 0; k < voMat.cols; k++)
			voMat.at<cv::Vec2f>(i, k) = cv::Vec2f(vMap.getValueAt(i, k)[0] * vCoeff, vMap.getValueAt(i, k)[1] * vCoeff);
}
