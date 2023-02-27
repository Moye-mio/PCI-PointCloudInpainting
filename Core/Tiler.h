#pragma once

#include "AABB.h"

namespace core
{
	class CTiler
	{
	public:
		CTiler() = default;
		~CTiler() = default;

		bool run(const PC_t::Ptr& vCloud, int vSizeX, int vSizeY, float vRate);
		void dumpResult(std::vector<std::vector<int>>& voTiles, std::vector<std::pair<int, int>>& voCoors) { voTiles = m_Tiles; voCoors = m_Coors; }

	private:
		bool __isInRange(const Eigen::Vector4f& vRange, const Eigen::Vector2f& vPos);

	private:
		std::vector<std::vector<int>>		m_Tiles;
		std::vector<std::pair<int, int>>	m_Coors;
	};
}