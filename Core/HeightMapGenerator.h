#pragma once
#include "HeightMap.h"
#include "AABB.h"

namespace core
{
	class CHeightMapGenerator
	{
	public:
		CHeightMapGenerator();
		~CHeightMapGenerator() = default;

		[[nodiscard]] bool setCloud(const PC_t::Ptr& vCloud);
		[[nodiscard]] bool setAABB(const SAABB& vBox);
		[[nodiscard]] bool generate(int vWidth, int vHeight);
		[[nodiscard]] bool generateBySurface(int vWidth, int vHeight);

		void dumpHeightMap(CHeightMap& voMap) const { voMap = m_Map; }
		
	private:
		Eigen::Vector2i __computeOffset(const Point_t& vPoint);


	private:
		CHeightMap m_Map;
		PC_t::Ptr m_pCloud;
		SAABB m_Box;

	};
}