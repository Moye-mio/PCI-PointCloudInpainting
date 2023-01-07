#include "pch.h"

#include "DataTrimmer.h"

using namespace core;

void CDataTrimmer::sort(const std::vector<Eigen::Vector3i>& vPos)
{
	_ASSERTE(vPos.size());

	std::vector<std::vector<unsigned int>> NeighInfo;
	__buildNeighbors(vPos, NeighInfo);
	__cullOutliers(NeighInfo);
	__trimByLayer(vPos, NeighInfo);
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
	
	//_ASSERTE(!std::isnan(voBB.first[0] + voBB.first[1] + voBB.first[2]) && !std::isnan(voBB.second[0] + voBB.second[1] + voBB.second[2]));
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

std::optional<unsigned int> CDataTrimmer::__chooseStartPoint(const std::vector<Eigen::Vector3i>& vPos, const std::vector<std::vector<unsigned int>>& vNeighInfo, const std::vector<bool>& vIsUsed)
{
	_ASSERTE(vPos.size() && vPos.size() == vIsUsed.size() && vPos.size() == vNeighInfo.size());

	
	

	for (int i = 0; i < vNeighInfo.size(); i++)
	{
		if (m_IsValid[i] == false || vIsUsed[i] == true) continue;


	}

	int MinSum = INT_MAX;
	std::vector<unsigned int> MinIndices;
	for (int i = 0; i < vPos.size(); i++)
	{
		if (m_IsValid[i] == false || vIsUsed[i] == true) continue;
		int Index = vPos[i].x() + vPos[i].y() + vPos[i].z();
		if (MinSum > Index)
		{
			MinSum = Index;
			MinIndices.clear();
			MinIndices.shrink_to_fit();
			MinIndices.push_back(i);
		}
		else if (MinSum == Index)
			MinIndices.push_back(i);
	}

	if (MinIndices.size() == 0)
		return std::nullopt;

	std::sort(MinIndices.begin(), MinIndices.end(),
		[&](unsigned int a, unsigned int b) -> bool
		{
			if (vPos[a].x() != vPos[b].x())		return vPos[a].x() < vPos[b].x();
			if (vPos[a].y() != vPos[b].y())		return vPos[a].y() < vPos[b].y();
			else								return vPos[a].z() < vPos[b].z();
		});

	return MinIndices[0];
}

void CDataTrimmer::__trimByLayer(const std::vector<Eigen::Vector3i>& vPos, const std::vector<std::vector<unsigned int>>& vNeighInfo)
{
	_ASSERTE(vPos.size() == vNeighInfo.size());

	std::vector<bool> IsTraversed(vNeighInfo.size(), false);
	std::vector<std::vector<unsigned int>> Sorted;
	
	while (true)
	{
		std::vector<unsigned int> UpdateList;
		unsigned int StartIndex;
		if (auto r = __chooseStartPoint(vPos, IsTraversed); r.has_value())
			StartIndex = r.value();
		else
			break;
		std::vector<unsigned int> Series;
		Series.push_back(StartIndex);

		int CurIndex = StartIndex;
		UpdateList.push_back(CurIndex);
		unsigned int CurNeighCount = 0;
		unsigned int NextIndex = INT_MAX;
		while (true)
		{
			_ASSERTE(CurIndex >= 0 && CurIndex <= vNeighInfo.size());
			_ASSERTE(vNeighInfo[CurIndex].size() > 0);
			
			CurNeighCount = __calcValidNeighCount(vNeighInfo, IsTraversed, CurIndex);
			if (CurNeighCount == 0) 
				break;
			
			int NextNeighCount = INT_MAX;
			NextIndex = INT_MAX;
			for (unsigned int e : vNeighInfo[CurIndex])
			{
				int NeighCount = __calcValidNeighCount(vNeighInfo, IsTraversed, e);

				auto Iter = std::find(UpdateList.begin(), UpdateList.end(), e);
				if (Iter != UpdateList.end()) continue;

				/* Trick */
				{
					if (UpdateList.size() == 1 && NeighCount == CurNeighCount) continue;
					if (UpdateList.size() == 2 && NeighCount < CurNeighCount) continue;
				}

				if (m_IsValid[e] == true && IsTraversed[e] == false && NextNeighCount > NeighCount)
				{
					NextIndex = e;
					NextNeighCount = NeighCount;
				}
			}
			
			_ASSERTE(NextNeighCount > 0);
			Series.push_back(NextIndex);
			UpdateList.push_back(NextIndex);
			CurIndex = NextIndex;
			
			if (CurNeighCount > NextNeighCount)
				break;
		}
		Sorted.emplace_back(Series);
		__updateTraversed(IsTraversed, UpdateList);
	}

	__transVec2Mat(Sorted);
}

void CDataTrimmer::__transVec2Mat(const std::vector<std::vector<unsigned int>>& vData)
{
	_ASSERTE(vData.size());
	int Rows = vData.size();
	int Cols = 0;	/* Do not use any number < 0 here */

	for (const auto& e : vData)
		Cols = (Cols > e.size()) ? Cols : e.size();
	_ASSERTE(Cols > 0);

	m_Sorted.resize(Rows, Cols);
	for (int i = 0; i < Rows; i++)
		for (int k = 0; k < Cols; k++)
			m_Sorted.coeffRef(i, k) = vData[i][k];
}

unsigned int CDataTrimmer::__calcValidNeighCount(const std::vector<std::vector<unsigned int>>& vNeighInfo, const std::vector<bool>& vIsTraversed, unsigned int vIndex)
{
	_ASSERTE(vIndex <= vNeighInfo.size());

	const auto& Neigh = vNeighInfo[vIndex];
	unsigned int Count = 0;
	for (unsigned int e : Neigh)
		if (vIsTraversed[e] == false)
			Count++;

	return Count;
}

void CDataTrimmer::__updateTraversed(std::vector<bool>& vioIsTraversed, const std::vector<unsigned int>& vUpdateList)
{
	_ASSERTE(vioIsTraversed.size() >= vUpdateList.size());

	for (const unsigned int e : vUpdateList)
	{
		_ASSERTE(vioIsTraversed[e] == false);
		vioIsTraversed[e] = true;
	}
}
