#pragma once

#include <pcl/kdtree/kdtree_flann.h>

namespace core
{
	class CRegionGrowing3D
	{
	public:
		CRegionGrowing3D(const PC_t::Ptr& vCloud);
		~CRegionGrowing3D();

		void Grow(float vRadius);
		void dumpGrowingResult(std::vector<std::vector<int>>& voGrowingSet);

	private:
		void __Grow(int vIndex, std::vector<int>& vioCluster);

	private:
		PC_t::Ptr m_pCloud;
		pcl::KdTreeFLANN<Point_t>::Ptr m_pKdTree;
		std::vector<std::vector<int>> m_GrowingSet;
		std::vector<bool> m_IsTraversed;

		float m_Radius;
	};
}