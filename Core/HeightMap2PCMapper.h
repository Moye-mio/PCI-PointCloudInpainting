#pragma once
#include "HeightMap.h"
#include "AABB.h"

namespace core
{
	class CHeightMap2PCMapper
	{
	public:
		CHeightMap2PCMapper() {}
		~CHeightMap2PCMapper() {}

		void map2PC(PC_t::Ptr& voCloud, const std::pair<CHeightMap, CHeightMap>& vMaps, const SAABB& vBox, int vPointNumberPerPixel);

	private:
		void __generateRandomPoints(int vNumber, std::vector<std::pair<float, float>>& voRandomPoints);
		void __map2NewPoints(PC_t::Ptr& voCloud, const std::vector<std::pair<float, float>>& vPoints);
		std::pair<float, float> __computeCoor(const std::pair<float, float>& vPoints, bool vTune);

	private:
		CHeightMap m_RawMap;
		CHeightMap m_FilledMap;
		SAABB m_Box;
		std::pair<float, float> m_SpanPerPixel;
	};
}