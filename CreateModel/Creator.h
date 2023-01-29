#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using Point_t = pcl::PointXYZRGBNormal;
using PC_t = pcl::PointCloud<Point_t>;

namespace creator
{
	class CCreator
	{
	public:
		CCreator() = default;
		~CCreator() = default;

		bool createConcave(PC_t::Ptr& voCloud);

	private:

	};
}