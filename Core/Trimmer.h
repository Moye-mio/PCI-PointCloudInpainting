#pragma once

namespace core
{
	enum class EProj : std::uint8_t
	{
		ProjX = 0,
		ProjY = 1,
		ProjZ = 2
	};


	class CTrimmer
	{
	public:
		CTrimmer();
		~CTrimmer() = default;

		[[nodiscard]] bool setData(const std::vector<std::pair<Eigen::Vector3i, Point_t>>& vData, float vScale);
		void fillAndTrim(enum class EProj vProj = EProj::ProjZ);
		void dumpSorted(Eigen::Matrix<Point_t, -1, -1>& voSorted) { voSorted = m_Sorted; }

	private:
		void __calcBaseSize();
		void __proj2Base(Eigen::Matrix<std::vector<unsigned int>, -1, -1>& voMap);
		void __fillEmpty(Eigen::Matrix<std::vector<unsigned int>, -1, -1>& voMap);
		void __ExtractNeighInfo(const Eigen::Matrix<std::vector<unsigned int>, -1, -1>& vMap, int vRow, int vCol, std::vector<unsigned int>& voNeighInfo);
		void __trim(const Eigen::Matrix<std::vector<unsigned int>, -1, -1>& vMap);
		unsigned int __fillEmptyElement(const std::vector<unsigned int>& vNeighInfo, int vRow, int vCol);
		Point_t __calcAveragePoint(const std::vector<Point_t>& vPoints);
		Point_t __calcAveragePoint(const std::vector<unsigned int>& vIndices);

	private:
		std::vector<std::pair<Eigen::Vector3i, Point_t>>	m_Data;
		Eigen::Matrix<Point_t, -1, -1>						m_Sorted;
		float												m_Scale;
		int													m_BaseWidth;
		int													m_BaseHeight;
	};
}