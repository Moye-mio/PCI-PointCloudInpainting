#pragma once

#include "MultilayerSurface.h"

namespace dataManagement
{
	class CTilesInpainting
	{
	public:
		CTilesInpainting() = default;
		~CTilesInpainting() = default;

		bool run(const PC_t::Ptr& vRaw, const PC_t::Ptr& vSub, int vSizeX, int vSizeY, float vRate);

	private:
		bool __extractCtrlpts(const ON_NurbsSurface& vNurbs, Eigen::Matrix<core::SPoint, -1, -1>& voCtrlpts);
		void __projPoints(const std::shared_ptr<pcl::on_nurbs::FittingSurface>& vFit, core::CMultilayerSurface* vSurface, const PC_t::Ptr& vCloud, std::vector<std::pair<float, Eigen::Vector2f>>& voData);
		void __tuneMapBoundary(core::CHeightMap& vioMask);
		bool __isMapNoHole(const core::CHeightMap& vMap);
	};
}