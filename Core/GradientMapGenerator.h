#pragma once
#include "GradientMap.h"
#include "HeightMap.h"

namespace core
{
	class CGradientMapGenerator
	{
	public:
		CGradientMapGenerator();
		~CGradientMapGenerator() = default;

		[[nodiscard]] bool generate(const CHeightMap& vHeightMap, bool vIsConservative = true);
		void dumpGradientMap(CGradientMap& voMap) const { voMap = m_Map; }

	private:
		std::optional<Eigen::Vector2f> __computeGradient(int vRow, int vCol);

	private:
		CGradientMap m_Map;
		CHeightMap m_HeightMap;
		bool m_IsConservative;
	};
}