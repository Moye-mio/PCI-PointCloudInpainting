#pragma once

#include "MultilayerSurface.h"

namespace dataManagement
{
	class CSurfaceGenerator
	{
	public:
		CSurfaceGenerator() = default;
		~CSurfaceGenerator() = default;

		bool run(const std::string& vPath);
		void dumpSurface(std::shared_ptr<core::CMultilayerSurface>& voSurface) { voSurface = m_pSurface; }
		void dumpControlPoints(Eigen::Matrix<Point_t, -1, -1>& voControlPoints) { voControlPoints = m_ControlPoints; }
		void dumpProjData(std::vector<std::pair<float, Eigen::Vector2f>>& voData) { voData = m_Data; }

	private:
		bool __isEmptyInVoxels(const Eigen::Matrix<Point_t, -1, -1>& vVoxels);
		bool __isPointSame(const Point_t& vLhs, const Point_t& vRhs);
		/*bool __*/
		void __castPCLPoint2SPoint(const Eigen::Matrix<Point_t, -1, -1>& vPCLPoints, Eigen::Matrix<core::SPoint, -1, -1>& voSPoints);
		void __extractProjData(core::CMultilayerSurface* vSurface, const PC_t::Ptr& vCloud, std::vector<std::pair<float, Eigen::Vector2f>>& voData);

	private:
		Eigen::Matrix<Point_t, -1, -1>					m_ControlPoints;
		std::shared_ptr<core::CMultilayerSurface>		m_pSurface;
		std::vector<std::pair<float, Eigen::Vector2f>>	m_Data;
	};
}