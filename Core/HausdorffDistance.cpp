#include "pch.h"
#include "HausdorffDistance.h"

#include <pcl/search/kdtree.h>

using namespace core;

std::optional<float> CHausDorffDistance::compute(const PC_t::Ptr& vLhs, const PC_t::Ptr& vRhs)
{
	_HIVE_EARLY_RETURN(vLhs == nullptr || vRhs == nullptr, "Cloud is nullptr", std::nullopt);
	_HIVE_EARLY_RETURN(vLhs->empty() || vRhs->empty(), "Cloud is empty", std::nullopt);

	pcl::search::KdTree<Point_t> TreeA, TreeB;
	TreeB.setInputCloud(vRhs);
	float MaxA = -FLT_MAX;
	for (int i = 0; i < vLhs->size(); i++)
	{
		std::vector<int> Indices(1);
		std::vector<float> SqrDist(1);

		TreeB.nearestKSearch(vLhs->at(i), 1, Indices, SqrDist);
		if (SqrDist[0] > MaxA)
			MaxA = SqrDist[0];
	}

	TreeA.setInputCloud(vLhs);
	float MaxB = -FLT_MAX;
	for (size_t i = 0; i < vRhs->size(); ++i)
	{
		std::vector<int> Indices(1);
		std::vector<float> SqrDist(1);

		TreeA.nearestKSearch(vRhs->at(i), 1, Indices, SqrDist);
		if (SqrDist[0] > MaxB)
			MaxB = SqrDist[0];
	}

	MaxA = std::sqrt(MaxA);
	MaxB = std::sqrt(MaxB);

	float Dist = std::max(MaxA, MaxB);
	return Dist;
}
