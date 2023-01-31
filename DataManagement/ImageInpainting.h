#pragma once

namespace dataManagement
{
	class CImageInpainting
	{
	public:
		CImageInpainting() = default;
		~CImageInpainting() = default;

		bool run(const core::CHeightMap& vMap, core::CHeightMap& voInpainted);

	private:
		void __castMap2CV(const core::CGradientMap& vMap, cv::Mat& voMat, float vCoeff);
		void __castMap2CV(const core::CHeightMap& vMap, cv::Mat& voMat, float vCoeff);
		void __castCV2Map(const cv::Mat& vMat, core::CGradientMap& voMap, float vCoeff);
		bool __setValueInMap(core::CHeightMap& vioMap, const std::vector<Eigen::Vector2f>& vUnknowns, const Eigen::MatrixXf& vSolutions);

	};
}