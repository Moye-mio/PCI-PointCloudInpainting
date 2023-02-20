#include "pch.h"
#include "SimilarityEstimator.h"

using namespace core;

std::optional<float> CSimilarityEstimator::compute(const PC_t::Ptr& vLhs, const PC_t::Ptr& vRhs, ESimilarityMode vMode)
{
	_HIVE_EARLY_RETURN(vLhs == nullptr || vRhs == nullptr, "Cloud is nullptr", std::nullopt);
	_HIVE_EARLY_RETURN(vLhs->empty() || vRhs->empty(), "Cloud is empty", std::nullopt);

	switch (vMode)
	{
	case core::ESimilarityMode::GPSNR:
		__calcGPSNR(vLhs, vRhs);

		break;
	case core::ESimilarityMode::NSHD:
		__calcNSHD(vLhs, vRhs);
		break;
	default:
		break;
	}
}

float CSimilarityEstimator::__calcGPSNR(const PC_t::Ptr& vLhs, const PC_t::Ptr& vRhs)
{
	
}

float CSimilarityEstimator::__calcNSHD(const PC_t::Ptr& vLhs, const PC_t::Ptr& vRhs)
{

}
