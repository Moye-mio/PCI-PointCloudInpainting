#pragma once

namespace core
{
	enum class ESimilarityMode
	{
		GPSNR = 0,
		NSHD = 1
	};

	class CSimilarityEstimator
	{
	public:
		CSimilarityEstimator() = default;
		~CSimilarityEstimator() = default;

		std::optional<float> compute(const PC_t::Ptr& vLhs, const PC_t::Ptr& vRhs, ESimilarityMode vMode);

	private:
		float __calcGPSNR(const PC_t::Ptr& vLhs, const PC_t::Ptr& vRhs);
		float __calcNSHD(const PC_t::Ptr& vLhs, const PC_t::Ptr& vRhs);
	};
}