#pragma once

namespace core
{
	class CBiasedVoxelization
	{
	public:
		CBiasedVoxelization();
		~CBiasedVoxelization() = default;

		[[nodiscard]] bool setCloud(const PC_t::Ptr& vCloud);
		[[nodiscard]] bool setAABB(const SAABB& vBox);
		[[nodiscard]] bool setDenoiseThres(std::uint32_t vThres);
		[[nodiscard]] bool generate(float vDist);
		void dumpVoxel(std::vector<std::pair<Eigen::Vector3i, Point_t>>& voVoxel) const { voVoxel = m_Voxel; }

	private:
		bool __isDistValid(float vDist);
		bool __init(std::vector<std::pair<Eigen::Vector3i, std::vector<int>>>& voVoxelList, float vDist);
		bool __biasAABB(float vDist);
		Eigen::Vector3i __computeOffset(const Point_t& vPoint, float vDist);

	private:
		std::vector<std::pair<Eigen::Vector3i, Point_t>>	m_Voxel;
		SAABB												m_Box;
		PC_t::Ptr											m_pCloud;
		Eigen::Vector3i										m_Scale;
		std::uint32_t										m_Thres;
	};


}