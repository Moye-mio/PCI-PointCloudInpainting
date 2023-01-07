#pragma once

namespace core
{
	class CDataTrimmer
	{
	public:
		CDataTrimmer() {}
		~CDataTrimmer() = default;

		void sort(const std::vector<Eigen::Vector3i>& vPos);
		void dumpSortedIndices(Eigen::Matrix<unsigned int, -1, -1>& voIndices) { voIndices = m_Sorted; }

	private:
		void __calcBB(const std::vector<Eigen::Vector3i>& vPos, std::pair<Eigen::Vector3i, Eigen::Vector3i>& voBB);
		void __buildNeighbors(const std::vector<Eigen::Vector3i>& vPos, std::vector<std::vector<unsigned int>>& voNeighInfo);
		void __cullOutliers(const std::vector<std::vector<unsigned int>>& vNeighInfo);
		std::optional<unsigned int> __chooseStartPoint(const std::vector<Eigen::Vector3i>& vPos, const std::vector<std::vector<unsigned int>>& vNeighInfo, const std::vector<bool>& vIsUsed);
		void __trimByLayer(const std::vector<Eigen::Vector3i>& vPos, const std::vector<std::vector<unsigned int>>& vNeighInfo);
		void __transVec2Mat(const std::vector<std::vector<unsigned int>>& vData);
		unsigned int __calcValidNeighCount(const std::vector<std::vector<unsigned int>>& vNeighInfo, const std::vector<bool>& vIsTraversed, unsigned int vIndex);
		void __updateTraversed(std::vector<bool>& vioIsTraversed, const std::vector<unsigned int>& vUpdateList);

	private:
		Eigen::Matrix<unsigned int, -1, -1> m_Sorted;
		std::vector<bool>					m_IsValid;
	};
}