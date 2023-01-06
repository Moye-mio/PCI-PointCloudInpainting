#include "pch.h"

#include "DataTrimmer.h"

using namespace core;

void CDataTrimmer::sort(const std::vector<Eigen::Vector3i>& vPos)
{
	_ASSERTE(vPos.size());

	std::vector<std::vector<unsigned int>> NeighInfo;
	__buildNeighbors(vPos, NeighInfo);
	__cullOutliers(NeighInfo);
	__trimByLayer(NeighInfo, __chooseStartPoint(vPos));
}

void CDataTrimmer::__calcBB(const std::vector<Eigen::Vector3i>& vPos, std::pair<Eigen::Vector3i, Eigen::Vector3i>& voBB)
{
	voBB = std::make_pair<Eigen::Vector3i, Eigen::Vector3i>({ INT_MAX, INT_MAX, INT_MAX }, { -INT_MAX, -INT_MAX, -INT_MAX });	// min, max
	for (const auto& e : vPos)
		for (int i = 0; i < e.size(); i++)
		{
			voBB.first[i] = (voBB.first[i] < e[i]) ? voBB.first[i] : e[i];
			voBB.second[i] = (voBB.second[i] > e[i]) ? voBB.second[i] : e[i];
		}
	
	_ASSERTE(!std::isnan(voBB.first[0] + voBB.first[1] + voBB.first[2]) && !std::isnan(voBB.second[0] + voBB.second[1] + voBB.second[2]));
}

void CDataTrimmer::__buildNeighbors(const std::vector<Eigen::Vector3i>& vPos, std::vector<std::vector<unsigned int>>& voNeighInfo)
{
	voNeighInfo.clear();
	voNeighInfo.shrink_to_fit();
	voNeighInfo.resize(vPos.size());

	std::pair<Eigen::Vector3i, Eigen::Vector3i> Box;
	__calcBB(vPos, Box);

	int Count = 0;
	for (const auto& e : vPos)
	{
		for (int i = e.x() - 1; i <= e.x() + 1; i++)
		{
			if (i < Box.first.x() || i > Box.second.x()) continue;
			for (int k = e.y() - 1; k <= e.y() + 1; k++)
			{
				if (k < Box.first.y() || k > Box.second.y()) continue;
				for (int m = e.z() - 1; m <= e.z() + 1; m++)
				{
					if (m < Box.first.z() || m > Box.second.z()) continue;
					auto Iter = std::find(vPos.begin(), vPos.end(), Eigen::Vector3i(i, k, m));
					if (Iter != vPos.end())
						voNeighInfo[Count].emplace_back(std::distance(vPos.begin(), Iter));
				}
			}
		}
		Count++;
	}
}

void CDataTrimmer::__cullOutliers(const std::vector<std::vector<unsigned int>>& vNeighInfo)
{
	for (const auto& e : vNeighInfo)
	{
		if (e.size() > 0)
			m_IsValid.emplace_back(true);
		else
			m_IsValid.emplace_back(false);
	}
}

unsigned int CDataTrimmer::__chooseStartPoint(const std::vector<Eigen::Vector3i>& vPos)
{
	_ASSERTE(vPos.size());

	int MinIndex = INT_MAX;
	std::vector<unsigned int> MinPoints;

	for (int i = 0; i < vPos.size(); i++)
	{
		if (m_IsValid[i] == false) continue;
		int Index = vPos[i].x() + vPos[i].y() + vPos[i].z();
		if (MinIndex > Index)
		{
			MinIndex = Index;
			MinPoints.clear();
			MinPoints.shrink_to_fit();
			MinPoints.push_back(i);
		}
		else if (MinIndex == Index)
			MinPoints.push_back(i);
	}

	std::sort(MinPoints.begin(), MinPoints.end(),
		[&](unsigned int a, unsigned int b) -> bool
		{
			if (vPos[a].x() != vPos[b].x())		return vPos[a].x() < vPos[b].x();
			if (vPos[a].y() != vPos[b].y())		return vPos[a].y() < vPos[b].y();
			else								return vPos[a].z() < vPos[b].z();
		});

	return MinPoints[0];
}

void CDataTrimmer::__trimByLayer(const std::vector<std::vector<unsigned int>>& vNeighInfo, unsigned int vStart)
{
	std::vector<bool> IsTraversed(vNeighInfo.size(), false);
	std::vector<std::vector<unsigned int>> Sorted;
	
	while (true)
	{
		std::vector<unsigned int> Temp;
		Temp.push_back(vStart);
	}

}

