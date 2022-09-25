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

std::optional<int> findPostion(const std::vector<Eigen::Vector2f>& vVec, const Eigen::Vector2f& vPos)
{
	auto Iter = std::find(vVec.begin(), vVec.end(), vPos);
	if (Iter == vVec.end()) return std::nullopt;
	else return std::distance(vVec.begin(), Iter);
}

//float dealWithBoundary(bool vIsKnown, float vValue, const Eigen::Vector2f& vPos, Eigen::MatrixXf& vA, int vEquationNumber, float vConst)
//{
//	if (vIsKnown)
//		return vConst - vValue;
//	else
//	{
//		if (auto r = findPostion(Unknowns, Eigen::Vector2f(i, k - 1)); r.has_value())
//			A.coeffRef(EquationNumber, r.value()) = 1;
//		else
//			return false;
//	}
//}

bool setUpEquations(const core::CHeightMap& vHeightMap, const core::CGradientMap& vGoG)
{
	core::CHeightMap Mask;
	vHeightMap.generateMask(Mask);
	std::vector<Eigen::Vector2f> Unknowns;
	for (int i = 0; i < Mask.getWidth(); i++)
		for (int k = 0; k < Mask.getHeight(); k++)
			if (Mask.getValueAt(i, k) == 1)
				Unknowns.push_back({ i, k });

	_ASSERTE(Unknowns.size());
	Eigen::MatrixXf A(Unknowns.size(), Unknowns.size()), B(Unknowns.size(), Unknowns.size());
	A.setZero();
	int EquationNumber = 0;
	for (int i = 0; i < Mask.getWidth(); i++)
		for (int k = 0; k < Mask.getWidth(); k++)
		{
			if (Mask.getValueAt(i, k) == 0) continue;
			_ASSERTE(Mask.getValueAt(i, k) == 1);
			float ConstNumber = vGoG.getValueAt(i, k)[0] + vGoG.getValueAt(i, k)[1];
			if ((i == 0 || i == Mask.getWidth() - 1) && (k == 0 || k == Mask.getHeight() - 1))			/* Corner */
			{

			}
			else if (i == 0 || i == Mask.getWidth() - 1 || k == 0 || k == Mask.getHeight() - 1)			/* Border */
			{

			}
			else
			{
				// D(x,y-1)+D(x-1,y)+D(x+1,y)+D(x,y+1)-4D(x,y)=GoG(x,y)[0]+GoG(x,y)[1]
				if (Mask.getValueAt(i, k - 1) == 0) 
					ConstNumber -= vHeightMap.getValueAt(i, k - 1);
				else
				{
					if (auto r = findPostion(Unknowns, Eigen::Vector2f(i, k - 1)); r.has_value())
						A.coeffRef(EquationNumber, r.value()) = 1;
					else
						return false;
				}

				if (Mask.getValueAt(i, k - 1) == 0)
					ConstNumber -= vHeightMap.getValueAt(i, k);
				else
				{
					if (auto r = findPostion(Unknowns, Eigen::Vector2f(i, k - 1)); r.has_value())
						A.coeffRef(EquationNumber, r.value()) = 1;
					else
						return false;
				}
			}
			EquationNumber++;
		}
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

