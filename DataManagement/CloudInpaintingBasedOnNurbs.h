#pragma once

#include "MultilayerSurface.h"

namespace dataManagement
{
	class CCloudInpaintingBasedOnNurbs
	{
	public:
		CCloudInpaintingBasedOnNurbs();
		~CCloudInpaintingBasedOnNurbs() = default;

		bool run(const PC_t::Ptr& vCloud);
		void dumpCloud(PC_t::Ptr& voCloud) { voCloud = m_pCloud; }

	private:
		bool __extractCtrlpts(const ON_NurbsSurface& vNurbs, Eigen::Matrix<core::SPoint, -1, -1>& voCtrlpts);
		void __projPoints(const std::shared_ptr<pcl::on_nurbs::FittingSurface>& vFit, core::CMultilayerSurface* vSurface, const PC_t::Ptr& vCloud, std::vector<std::pair<float, Eigen::Vector2f>>& voData);
		void __tuneMapBoundary(core::CHeightMap& vioMask);
		bool __map2Cloud(const core::CHeightMap& vMask, const core::CHeightMap& vInpainted, int vSPP, const std::shared_ptr<pcl::on_nurbs::FittingSurface>& vFit, const std::shared_ptr<core::CMultilayerSurface>& vSurface, PC_t::Ptr& vCloud);

	private:
		PC_t::Ptr m_pCloud;

	};
}