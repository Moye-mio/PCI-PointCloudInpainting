#pragma once

namespace core
{
	class CControlPointGenerator
	{
	public:
		CControlPointGenerator() = default;
		~CControlPointGenerator() = default;

		bool run(const Eigen::Matrix<Point_t, -1, -1>& vData);
		void dumpControlPoints(Eigen::Matrix<Point_t, -1, -1>& voPoints) { voPoints = m_Points; }

	private:
		Eigen::Matrix<Point_t, -1, -1> m_Points;
	};
}