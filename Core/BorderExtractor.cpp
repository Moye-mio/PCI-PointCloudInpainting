#include "pch.h"

#include "BorderExtractor.h"

#include <numbers>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace core;

CBorderExtractor::CBorderExtractor(const PC_t::Ptr& vCloud)
	: m_pCloud(vCloud)
{}

void CBorderExtractor::Compute(float vRadius)
{
	_ASSERTE(m_pCloud != nullptr);
	_ASSERTE(m_pCloud->size());
	_ASSERTE(vRadius > 0);

	pcl::PointCloud<pcl::Boundary> Boundaries;
	pcl::BoundaryEstimation<Point_t, pcl::Normal, pcl::Boundary> BoundEst;
	pcl::NormalEstimation<Point_t, pcl::Normal> NormEst;
	pcl::PointCloud<pcl::Normal>::Ptr Normals(new pcl::PointCloud<pcl::Normal>);
	NormEst.setInputCloud(m_pCloud);
	NormEst.setRadiusSearch(vRadius);
	NormEst.compute(*Normals);

	BoundEst.setInputCloud(m_pCloud);
	BoundEst.setInputNormals(Normals);
	BoundEst.setRadiusSearch(vRadius);
	BoundEst.setAngleThreshold(std::numbers::pi / 4);
	BoundEst.setSearchMethod(pcl::search::KdTree<Point_t>::Ptr(new pcl::search::KdTree<Point_t>));
	BoundEst.compute(Boundaries);

	for (int i = 0; i < m_pCloud->points.size(); i++)
		if (Boundaries[i].boundary_point > 0)
			m_BorderIndices.push_back(i);
}

