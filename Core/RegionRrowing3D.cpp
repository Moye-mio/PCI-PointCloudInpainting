#include "pch.h"

#include "RegionRrowing3D.h"

using namespace core;

CRegionGrowing3D::CRegionGrowing3D(const PC_t::Ptr& vCloud)
	: m_pCloud(vCloud)
{}

CRegionGrowing3D::~CRegionGrowing3D()
{}

void CRegionGrowing3D::Grow(float vRadius)
{
	_ASSERTE(m_pCloud != NULL);
	_ASSERTE(m_pCloud->size());
	_ASSERTE(vRadius);

	m_Radius = vRadius;
	m_pKdTree = std::make_shared<pcl::KdTreeFLANN<Point_t>>();
	m_pKdTree->setInputCloud(m_pCloud);

	m_IsTraversed.resize(m_pCloud->size(), false);
	for (int i = 0; i < m_pCloud->size(); i++)
	{
		if (m_IsTraversed[i] == true) continue;

		std::vector<int> Cluster;
		m_IsTraversed[i] = true;
		Cluster.push_back(i);
		__Grow(i, Cluster);

		m_GrowingSet.emplace_back(Cluster);
	}
}

void CRegionGrowing3D::__Grow(int vIndex, std::vector<int>& vioCluster)
{
	_ASSERTE(!vioCluster.empty());

	// Depth first
	std::uint32_t SeedIndex = 0;
	while (true)
	{
		if (SeedIndex == vioCluster.size()) break;
		int CurrentIndex = vioCluster[SeedIndex];

		std::vector<int> Indices;
		std::vector<float> Distances;
		m_pKdTree->radiusSearch(m_pCloud->at(CurrentIndex), m_Radius, Indices, Distances);

		for (auto e : Indices)
		{
			if (m_IsTraversed[e] == true) continue;
			m_IsTraversed[e] = true;
			vioCluster.push_back(e);
		}

		SeedIndex++;
	}
}

void core::CRegionGrowing3D::dumpGrowingResult(std::vector<std::vector<int>>& voGrowingSet)
{
	voGrowingSet = m_GrowingSet;
}

