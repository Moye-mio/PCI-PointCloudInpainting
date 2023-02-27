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
	_HIVE_EARLY_RETURN(vCloud == NULL, "Normal Estimator: Cloud is null", false);
	_HIVE_EARLY_RETURN(vCloud->size() == 0, "Normal Estimator: Cloud is empty", false);

	m_Cloud = vCloud;
	return true;
}

bool CNormalEstimator::compute(float vRadius)
{
	_HIVE_EARLY_RETURN(m_Cloud == NULL, "Normal Estimator: Cloud is null", false);
	_HIVE_EARLY_RETURN(m_Cloud->size() == 0, "Normal Estimator: Cloud is empty", false);
	_HIVE_EARLY_RETURN(vRadius <= 0.0f, "Normal Estimator: Radius < 0", false);

	pcl::NormalEstimation<Point_t, Normal_t> Estimator;
	Estimator.setInputCloud(m_Cloud);
	pcl::search::KdTree<Point_t>::Ptr KdTree(new pcl::search::KdTree<Point_t>());
	Estimator.setSearchMethod(KdTree);
	Estimator.setRadiusSearch(vRadius);
	Estimator.compute(*m_Normals);

	bool r = __validateNormal();
	return r;
}

bool CNormalEstimator::compute(int vK)
{
	_HIVE_EARLY_RETURN(m_Cloud == NULL, "Normal Estimator: Cloud is null", false);
	_HIVE_EARLY_RETURN(m_Cloud->size() == 0, "Normal Estimator: Cloud is empty", false);
	_HIVE_EARLY_RETURN(vK <= 0, "Normal Estimator: K < 0", false);

	pcl::NormalEstimation<Point_t, Normal_t> Estimator;
	Estimator.setInputCloud(m_Cloud);
	pcl::search::KdTree<Point_t>::Ptr KdTree(new pcl::search::KdTree<Point_t>());
	Estimator.setSearchMethod(KdTree);
	Estimator.setKSearch(vK);
	Estimator.compute(*m_Normals);

	bool r = __validateNormal();
	return r;
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

	vioNormal = Normal_t(0, 0, 0, Normal[0], Normal[1], Normal[2]);
	return true;
}

bool CNormalEstimator::__validateNormal()
{
	int Number = 0;
	for (auto& e : *m_Normals)
	{
		if (std::isnan(e.normal_x) || std::isnan(e.normal_y) || std::isnan(e.normal_z) || std::isinf(e.normal_x) || std::isinf(e.normal_y) || std::isinf(e.normal_z)) Number++;

		if (std::isnan(e.normal_x)) e.normal_x = 0.0f;
		if (std::isnan(e.normal_y)) e.normal_y = 0.0f;
		if (std::isnan(e.normal_z)) e.normal_z = 0.0f;

		if (std::isinf(e.normal_x)) e.normal_x = 1.0f;
		if (std::isinf(e.normal_y)) e.normal_y = 1.0f;
		if (std::isinf(e.normal_z)) e.normal_z = 1.0f;

		if (e.normal_x == 0 && e.normal_y == 0 && e.normal_z == 0) e.normal_x = 1.0f;

		_HIVE_EARLY_RETURN(__normalize(e) == false, _FORMAT_STR3("PCL Normal Estimator: (%1%, %2%, %3%) normalize failed", e.normal_x, e.normal_y, e.normal_z), false);
	}

	hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("Normal Invalid Number: %1%", Number));
	return true;
}
