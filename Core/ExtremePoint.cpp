#include "pch.h"

#include "ExtremePoint.h"
#include "AABBEstimation.h"

#include <unordered_map>

using namespace core;

CExtremePoint::CExtremePoint(const PC_t::Ptr& vCloud)
	: m_pCloud(vCloud)
{

}

bool CExtremePoint::compute()
{
	__preprocess();

	int Index = 0;
	for (auto& e : *m_pCloud)
	{
		if (e.x == m_Box._Max[0] || e.x == m_Box._Min[0] || e.y == m_Box._Max[1] || e.y == m_Box._Min[1] || e.z == m_Box._Max[2] || e.z == m_Box._Min[2])
			m_ExtremeId.push_back(Index);
		Index++;
	}

	if (m_ExtremeId.size() == 0)
		return false;

	return true;
}

bool CExtremePoint::setAABB(const SAABB& vBox)
{
	_ASSERTE(vBox.isValid());
	m_Box = vBox;

	return true;
}

void CExtremePoint::dumpPoints(std::vector<Point_t>& voPoints) const
{
	if (voPoints.size())
	{
		voPoints.clear();
		voPoints.shrink_to_fit();
	}

	for (auto e : m_ExtremeId)
		voPoints.push_back(m_pCloud->at(e));
}

bool CExtremePoint::computeExtreme()
{
	_ASSERTE(__preprocess());

	int Index = 0;
	std::pair<float, std::vector<int>> MaxPair, MinPair;
	MaxPair = std::make_pair(-FLT_MAX, std::vector<int>());
	MinPair = std::make_pair(FLT_MAX, std::vector<int>());
	for (auto& e : *m_pCloud)
	{
		float Sum = e.x + e.y + e.z;
		if (Sum > MaxPair.first)
		{
			MaxPair.first = Sum;
			MaxPair.second.clear();
			MaxPair.second.push_back(Index);
		}
		else if (Sum == MaxPair.first)
		{
			MaxPair.second.push_back(Index);
		}
		if (Sum < MinPair.first)
		{
			MinPair.first = Sum;
			MinPair.second.clear();
			MinPair.second.push_back(Index);
		}
		else if (Sum == MinPair.first)
		{
			MinPair.second.push_back(Index);
		}

		Index++;
	}

	for (auto& e : MaxPair.second)
		m_ExtremePointId.push_back(e);
	for (auto& e : MinPair.second)
		m_ExtremePointId.push_back(e);

	return true;
}

bool CExtremePoint::__preprocess()
{
	_ASSERTE(m_pCloud != nullptr);
	_ASSERTE(m_pCloud->size());

	if (!m_Box.isValid())
	{
		core::CAABBEstimation Estimation(m_pCloud);
		m_Box = Estimation.compute();
		_ASSERTE(m_Box.isValid());
	}
	return true;
}

void CExtremePoint::dumpExtremePoints(std::vector<Point_t>& voPoints) const
{
	if (voPoints.size())
	{
		voPoints.clear();
		voPoints.shrink_to_fit();
	}

	for (auto e : m_ExtremePointId)
		voPoints.push_back(m_pCloud->at(e));
}
