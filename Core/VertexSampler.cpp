#include "pch.h"
#include "VertexSampler.h"

using namespace core;

bool CVertexSampler::setSurface(const std::shared_ptr<core::CMultilayerSurface>& vSurface)
{
	_HIVE_EARLY_RETURN(vSurface->IsPreComputed() == false, "ERROR: Vertex Sampler: Surface has not Pre Computed", false);
	m_pSurface = vSurface;
	return true;
}

std::optional<core::SVertex> CVertexSampler::sample(const Eigen::Vector2f& vUV)
{
	_HIVE_EARLY_RETURN(m_pSurface->IsPreComputed() == false, "ERROR: Vertex Sampler: Surface has not Pre Computed", std::nullopt);
	_HIVE_EARLY_RETURN(std::isnan(vUV[0]) || std::isnan(vUV[1]), "ERROR: Vertex Sampler: UV is nan", std::nullopt);

	auto r =__sample(vUV);
	_HIVE_EARLY_RETURN(r.has_value() == false, "ERROR: Vertex Sampler: Sample Failed", std::nullopt);
	return r.value();
}

bool CVertexSampler::sample(const std::vector<Eigen::Vector2f>& vUVs, std::vector<SVertex>& voSamples)
{
	_HIVE_EARLY_RETURN(m_pSurface->IsPreComputed() == false, "ERROR: Vertex Sampler: Surface has not Pre Computed", false);
	_HIVE_EARLY_RETURN(vUVs.size() == 0, "ERROR: Vertex Sampler: Vector Size 0", false);
	
	int Count = -1;
	for (const auto& e : vUVs)
	{
		Count++;
		_HIVE_EARLY_RETURN(std::isnan(e[0]) || std::isnan(e[1]), _FORMAT_STR3("ERROR: Vertex Sampler: UV Number [%1%] is nan, Value: [%2%], [%3%]", Count, e[0], e[1]), false);

		auto r = __sample(e);
		_HIVE_EARLY_RETURN(r.has_value() == false, "ERROR: Vertex Sampler: Sample Failed", false);
		voSamples.emplace_back(r.value());
	}
	
	_HIVE_EARLY_RETURN(vUVs.size() != voSamples.size(), "ERROR: Vertex Sampler: Unexpected ERROR", false);
	return true;
}

std::optional<SVertex> CVertexSampler::__sample(const Eigen::Vector2f& vUV)
{
	return m_pSurface->sample(vUV);
}
