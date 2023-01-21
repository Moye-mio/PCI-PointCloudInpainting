#pragma once

#include "HeightMap.h"

namespace core
{
	class CDistSampler
	{
	public:
		CDistSampler() = default;
		~CDistSampler() = default;

		bool setResource(const core::CHeightMap& vRes);
		bool samplebyUV(const std::vector<Eigen::Vector2f>& vUVs, std::vector<float>& voDists);

	private:
		core::CHeightMap m_Res;
	};
}