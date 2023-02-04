#include "pch.h"

#include "Sorter.h"

using namespace core;

bool CSorter::sort(const std::vector<std::pair<Eigen::Vector3i, Point_t>>& vVoxels)
{
	_HIVE_EARLY_RETURN(vVoxels.size() == 0, "Sorter: Voxel is empty", false);

	Eigen::Vector3i Min, Max;
	_HIVE_EARLY_RETURN(__calcBB(vVoxels, Min, Max) == false, "Sorter: calc BB failed", false);

	Eigen::Matrix<std::vector<int>, -1, -1> Indices;
	Indices.resize(Max[0] - Min[0] + 1, Max[1] - Min[1] + 1);

	for (int i = 0; i < vVoxels.size(); i++)
	{
		const auto Pos = vVoxels[i].first;
		int OffsetX = Pos[0] - Min[0];
		int OffsetY = Pos[1] - Min[1];
		_HIVE_EARLY_RETURN(OffsetX < 0 || OffsetX > Max[0] - Min[0] || OffsetY < 0 || OffsetY > Max[1] - Min[1], "Sorter: overflow", false);

		Indices.coeffRef(OffsetX, OffsetY).emplace_back(i);
	}

	m_Voxels.resize(Indices.rows(), Indices.cols());
	for (int i = 0; i < Indices.rows(); i++)
		for (int k = 0; k < Indices.cols(); k++)
		{
			const std::vector<int>& Node = Indices.coeff(i, k);
			if (Node.size() == 0)
			{
				m_Voxels.coeffRef(i, k) = Point_t(-FLT_MAX, -FLT_MAX, -FLT_MAX);
				continue;
			}

			Point_t Point(0, 0, 0);
			for (auto e : Node)
			{
				Point.x += vVoxels[e].second.x;
				Point.y += vVoxels[e].second.y;
				Point.z += vVoxels[e].second.z;
			}

			Point.x /= Node.size();
			Point.y /= Node.size();
			Point.z /= Node.size();

			m_Voxels.coeffRef(i, k) = Point;
		}
	
	return true;
}

bool CSorter::__calcBB(const std::vector<std::pair<Eigen::Vector3i, Point_t>>& vVoxels, Eigen::Vector3i& voMin, Eigen::Vector3i& voMax)
{
	_HIVE_EARLY_RETURN(vVoxels.size() == 0, "Sorter: Voxel is empty", false);

	voMin = Eigen::Vector3i(INT_MAX, INT_MAX, INT_MAX);
	voMax = Eigen::Vector3i(-INT_MAX, -INT_MAX, -INT_MAX);

	for (const auto& e : vVoxels)
		for (int i = 0; i < 3; i++)
		{
			voMin[i] = (voMin[i] < e.first[i]) ? voMin[i] : e.first[i];
			voMax[i] = (voMax[i] > e.first[i]) ? voMax[i] : e.first[i];
		}

	_HIVE_EARLY_RETURN(voMin[0] > voMax[0] || voMin[1] > voMax[1] || voMin[2] > voMax[2], _FORMAT_STR6("Sorter: BB is Invalid, Min: [%1%, %2%, %3%], Max: [%4%, %5%, %6%]", voMin[0], voMin[1], voMin[2], voMax[0], voMax[1], voMax[2]), false);
	return true;
}

