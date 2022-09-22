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
		[[nodiscard]] bool generate(const CGradientMap& vGradientMap, bool vIsConservative = true);
		void dumpGradientMap(CGradientMap& voMap) const { voMap = m_Map; }

	private:
		bool __generate(const CHeightMap& vHeightMap, int vAxis);
		std::optional<float> __computeGradient(const CHeightMap& vHeightMap, int vRow, int vCol, int vMode);

	private:
		CGradientMap m_Map;
		bool m_IsConservative;
	};
}