#pragma once
#include "MultilayerSurface.h"

namespace core
{
	class CVertexSampler
	{
	public:
		CVertexSampler() = default;
		~CVertexSampler() = default;

		bool setSurface(const std::shared_ptr<core::CMultilayerSurface>& vSurface);
		std::optional<SVertex> sample(const Eigen::Vector2f& vUV);
		bool sample(const std::vector<Eigen::Vector2f>& vUVs, std::vector<SVertex>& voSamples);
		
	private:
		std::optional<SVertex> __sample(const Eigen::Vector2f& vUV);

	private:
		std::shared_ptr<core::CMultilayerSurface>	m_pSurface;

	};
}