#pragma once

namespace core
{
	class CDataTrimmer
	{
	public:
		CDataTrimmer() {}
		~CDataTrimmer() = default;

		void sort(const std::vector<Eigen::Vector3i>& vPos);
		void dumpSortedIndices(Eigen::Matrix<unsigned int, -1, -1>& voIndices);

	private:
		void __calcBB(const std::vector<Eigen::Vector3i>& vPos, std::pair<Eigen::Vector3i, Eigen::Vector3i>& voBB);
		void __buildNeighbors(const std::vector<Eigen::Vector3i>& vPos, std::vector<std::vector<unsigned int>>& voNeighInfo);
		void __cullOutliers(const std::vector<std::vector<unsigned int>>& vNeighInfo);
		unsigned int __chooseStartPoint(const std::vector<Eigen::Vector3i>& vPos);
		void __trimByLayer(const std::vector<std::vector<unsigned int>>& vNeighInfo, unsigned int vStart);

	private:
		Eigen::Matrix<unsigned int, -1, -1> m_Sorted;
		std::vector<bool>					m_IsValid;
	};
}