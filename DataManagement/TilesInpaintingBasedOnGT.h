#pragma once

#include "MultilayerSurface.h"

namespace dataManagement
{
	struct STilesInfo
	{
		float	_Rate;
		int		_SizeX;
		int		_SizeY;
	};

	struct SNurbsInfo
	{
		int	_Degree;
		int	_Refinement;
		int	_Iteration;
		int	_SubNumber;
		int _SubLayer;
	};

	struct SMapInfo
	{
		int _Width;
		int _Height;
		int _SPP;
	};

	class CTilesInpaintingBasedOnGT
	{
	public:
		CTilesInpaintingBasedOnGT();
		~CTilesInpaintingBasedOnGT() = default;

		bool setTilesInfo(float vRate, int vSizeX, int vSizeY);
		bool setNurbsInfo(int vDegree, int vRefinement, int vIteration, int vSubNumber, int vSubLayer);
		bool setMapInfo(int vWidth, int vHeight, int vSPP);
		bool run(const PC_t::Ptr& vRaw, const PC_t::Ptr& vSub, const PC_t::Ptr& vGT);
		void dumpCloud(PC_t::Ptr& vCloud) { vCloud = m_pCloud; }

	private:
		bool __setupTiles(const PC_t::Ptr& vCloud, Eigen::Matrix<PC_t::Ptr, -1, -1>& voTiles);
		bool __extractCtrlpts(const ON_NurbsSurface& vNurbs, Eigen::Matrix<core::SPoint, -1, -1>& voCtrlpts);
		void __projPoints(const std::shared_ptr<pcl::on_nurbs::FittingSurface>& vFit, core::CMultilayerSurface* vSurface, const PC_t::Ptr& vCloud, std::vector<std::pair<float, Eigen::Vector2f>>& voData);
		void __tuneMapBoundary(const core::CHeightMap& vGT, core::CHeightMap& vioRaw);
		bool __isMaskNoHole(const core::CHeightMap& vMap);
		void __saveImage(const core::CHeightMap& vMap, const std::string& vPath, int vCoeff);

	private:
		PC_t::Ptr	m_pCloud;

		STilesInfo	m_TilesInfo;
		SNurbsInfo	m_NurbsInfo;
		SMapInfo	m_MapInfo;
	};
}