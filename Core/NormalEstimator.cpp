#include "pch.h"
#include "NormalEstimator.h"

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace core;

CNormalEstimator::CNormalEstimator()
	: m_Normals(new NormalPC_t)
{}

bool CNormalEstimator::setCloud(const PC_t::Ptr& vCloud)
{
	_ASSERTE(vCloud != NULL);
	_ASSERTE(vCloud->size());

	m_Cloud = vCloud;

	return true;
}

void CNormalEstimator::compute(float vRadius)
{
	_ASSERTE(m_Cloud != NULL);
	_ASSERTE(m_Cloud->size());
	_ASSERTE(vRadius);

	pcl::NormalEstimation<Point_t, Normal_t> PCLEstimator;
	PCLEstimator.setInputCloud(m_Cloud);
	pcl::search::KdTree<Point_t>::Ptr KdTree(new pcl::search::KdTree<Point_t>());
	PCLEstimator.setSearchMethod(KdTree);
	PCLEstimator.setRadiusSearch(vRadius);
	PCLEstimator.compute(*m_Normals);

	for (auto& e : *m_Normals)
	{
		if (std::isnan(e.normal_x)) e.normal_x = 0.0f;
		if (std::isnan(e.normal_y)) e.normal_y = 0.0f;
		if (std::isnan(e.normal_z)) e.normal_z = 0.0f;
		if (e.normal_x == 0 && e.normal_y == 0 && e.normal_z == 0) e.normal_x = 1.0f;

		_HIVE_EARLY_RETURN(__normalize(e) == false, _FORMAT_STR3("PCL Normal Estimator: (%1%, %2%, %3%) normalize failed", e.normal_x, e.normal_y, e.normal_z), );
	}
}

bool CNormalEstimator::__normalize(Normal_t& vioNormal)
{
	float Epsilon = 0.0001f;
	Eigen::Vector3f Normal(vioNormal.normal_x, vioNormal.normal_y, vioNormal.normal_z);
	if (std::fabsf(Normal.norm() - 1.0f) >= Epsilon)
	{
		float SumOfSquares = Normal[0] * Normal[0] + Normal[1] * Normal[1] + Normal[2] * Normal[2];
		if (SumOfSquares == 0.0f)
			return false;

		Normal /= std::sqrtf(SumOfSquares);
	}

	vioNormal = Normal_t(Normal[0], Normal[1], Normal[2]);
	return true;
}
