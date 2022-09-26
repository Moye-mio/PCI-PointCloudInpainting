#pragma once

namespace dataManagement
{
	class CDepthInpaiting
	{
	public:
		CDepthInpaiting() {}
		~CDepthInpaiting() = default;

		void run(const PC_t::Ptr& vCloud, PC_t::Ptr& voResultCloud);

	private:
		void __castMap2CV(const core::CGradientMap& vMap, cv::Mat& voMat, float vCoeff);
		void __castMap2CV(const core::CHeightMap& vMap, cv::Mat& voMat, float vCoeff);
		void __castCV2Map(const cv::Mat& vMat, core::CGradientMap& voMap, float vCoeff);
		void __setValueInMap(core::CHeightMap& vioMap, const std::vector<Eigen::Vector2f>& vUnknowns, const Eigen::MatrixXf& vSolutions);
	};
}