#include "pch.h"

bool isPointSame(const Point_t& vLhs, const Point_t& vRhs)
{
	if (vLhs.x == vRhs.x && vLhs.y == vRhs.y && vLhs.z == vRhs.z && vLhs.r == vRhs.r && vLhs.g == vRhs.g && vLhs.b == vRhs.b && vLhs.a == vRhs.a)
		return true;
	else
		return false;
}

TEST(SortVoxels, DT_EmptyVector) {
	core::CSorter Sorter;
	EXPECT_FALSE(Sorter.sort(std::vector<std::pair<Eigen::Vector3i, Point_t>>()));
}

TEST(SortVoxels, NT_SimplePlane)
{
	std::vector<std::pair<Eigen::Vector3i, Point_t>> Data;
	Data.emplace_back(std::make_pair(Eigen::Vector3i(0, 0, 0), Point_t(0, 0, 0)));

	Eigen::Matrix<Point_t, -1, -1> SortedData;
	core::CSorter Sorter;
	EXPECT_TRUE(Sorter.sort(Data));
	Sorter.dumpSortedData(SortedData);
	EXPECT_TRUE(SortedData.size() == 1);
	EXPECT_TRUE(isPointSame(SortedData.coeff(0, 0), Point_t(0, 0, 0)));
}

TEST(SortVoxels, NT_MultiLayer)
{
	std::vector<std::pair<Eigen::Vector3i, Point_t>> Data;
	for (int i = 0; i < 3; i++)
		Data.emplace_back(std::make_pair(Eigen::Vector3i(0, 0, i), Point_t(0, 0, i)));

	Eigen::Matrix<Point_t, -1, -1> SortedData;
	core::CSorter Sorter;
	EXPECT_TRUE(Sorter.sort(Data));
	Sorter.dumpSortedData(SortedData);
	EXPECT_TRUE(SortedData.size() == 1);
	EXPECT_TRUE(isPointSame(SortedData.coeff(0, 0), Point_t(0, 0, 1)));
}

TEST(SortVoxels, NT_4x4x4MultiLayer)
{
	std::vector<std::pair<Eigen::Vector3i, Point_t>> Data;
	for (int i = 0; i < 4; i++)
		for (int k = 0; k < 4; k++)
			for (int m = 0; m < 4; m++)
				Data.emplace_back(std::make_pair(Eigen::Vector3i(m, k, i), Point_t(m, k, i)));

	Eigen::Matrix<Point_t, -1, -1> SortedData;
	core::CSorter Sorter;
	EXPECT_TRUE(Sorter.sort(Data));
	Sorter.dumpSortedData(SortedData);
	EXPECT_TRUE(SortedData.size() == 16);
	for (int i = 0; i < 4; i++)
		for (int k = 0; k < 4; k++)
			EXPECT_TRUE(isPointSame(SortedData.coeff(i, k), Point_t(i, k, 1.5f)));
}