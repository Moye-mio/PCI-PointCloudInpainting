#include "pch.h"
#include "DistSampler.h"

using namespace core;

bool CDistSampler::setResource(const core::CHeightMap& vRes)
{
	_HIVE_EARLY_RETURN(vRes.isValid() == false, "ERROR: Dist Sampler: Res is not Valid", false);
	m_Res = vRes;
	return true;
}

bool CDistSampler::samplebyUV(const std::vector<Eigen::Vector2f>& vUVs, std::vector<float>& voDists)
{
	_HIVE_EARLY_RETURN(vUVs.size() == 0, "ERROR: Dist Sampler: UVs is Empty", false);

	std::vector<float> Dists;
	int Count = -1;
	for (const auto& e : vUVs)
	{
		Count++;
		_HIVE_EARLY_RETURN(std::isnan(e[0]) && std::isnan(e[1]), _FORMAT_STR1("ERROR: Dist Sampler: UV Number [%1%] is nan", Count), false);

		float d = m_Res.Sample(std::make_pair(e[0], e[1]));
		_HIVE_EARLY_RETURN(std::isnan(d), _FORMAT_STR1("ERROR: Dist Sampler: Dist Number [%1%] is nan", Count), false);
		Dists.push_back(d);
	}

	voDists = Dists;
}

