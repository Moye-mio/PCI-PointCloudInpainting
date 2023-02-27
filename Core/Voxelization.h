#pragma once
#include "AABB.h"

namespace core
{
	class CVoxelization
	{
	public:
		CVoxelization();
		~CVoxelization() {}

		[[nodiscard]] bool setCloud(const PC_t::Ptr& vCloud);
		[[nodiscard]] bool setAABB(const SAABB& vBox);
		[[nodiscard]] bool generate(float vDist);
		void dumpVoxel(std::vector<std::pair<Eigen::Vector3i, Point_t>>& voVoxel) const { voVoxel = m_Voxel; }

	private:
		void __init(std::vector<std::pair<Eigen::Vector3i, std::vector<int>>>& voVoxelList, float vDist);
		Eigen::Vector3i __computeOffset(const Point_t& vPoint, float vDist);

	private:
		std::vector<std::pair<Eigen::Vector3i, Point_t>>	m_Voxel;
		PC_t::Ptr											m_pCloud;
		SAABB												m_Box;
		Eigen::Vector3i										m_Scale;
	};
}