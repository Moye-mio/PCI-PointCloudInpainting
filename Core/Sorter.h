#pragma once

namespace core
{
	class CSorter
	{
	public:
		CSorter() = default;
		~CSorter() = default;

		bool sort(const std::vector<std::pair<Eigen::Vector3i, Point_t>>& vVoxels);
		void dumpSortedData(Eigen::Matrix<Point_t, -1, -1>& voData) { voData = m_Voxels; }

	private:
		bool __calcBB(const std::vector<std::pair<Eigen::Vector3i, Point_t>>& vVoxels, Eigen::Vector3i& voMin, Eigen::Vector3i& voMax);

	private:
		Eigen::Matrix<Point_t, -1, -1>	m_Voxels;
	};
}