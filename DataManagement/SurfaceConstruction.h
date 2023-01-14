#pragma once

namespace dataManagement
{
	class CSurfaceConstruction
	{
	public:
		CSurfaceConstruction() {}
		~CSurfaceConstruction() = default;

		bool run(const PC_t::Ptr& vCloud, PC_t::Ptr& voResultCloud);

	private:
		void __transPCLPoints2SPoints(const Eigen::Matrix<Point_t, -1, -1>& vP, Eigen::Matrix<core::SPoint, -1, -1>& voS);
		core::SPoint __transPCLPoint2SPoint(const Point_t& vP);
		void __saveHeightMap(const core::CHeightMap& vMap);

	private:
		core::CHeightMap m_Map;

	};
}