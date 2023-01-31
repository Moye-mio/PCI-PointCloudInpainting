#include "pch.h"
#include "Surface2CloudMapper.h"
#include "SurfaceUVGenerator.h"
#include "VertexSampler.h"
#include "NormalSampler.h"
#include "DistSampler.h"
#include "Surface2PCMapper.h"

using namespace dataManagement;

CSurface2CloudMapper::CSurface2CloudMapper()
	: m_NewCloud(new PC_t)
{}

bool CSurface2CloudMapper::setSurface(const std::shared_ptr<core::CMultilayerSurface>& vSurface)
{
	_HIVE_EARLY_RETURN(vSurface->IsPreComputed() == false, "ERROR: Surface 2 Cloud Mapper: Surface has not Pre Computed", false);
	m_pSurface = vSurface;
	return true;
}

bool CSurface2CloudMapper::map2Cloud(const core::CHeightMap& vRaw, const core::CHeightMap& vMask, const core::CHeightMap& vInpainted, int vSPP)
{
	core::CHeightMap Mask = vMask;
	if (!__ValidCheck(vRaw, vInpainted, vSPP)) return false;
	if (!vMask.isValid()) 
		vRaw.generateMask(Mask);

	_HIVE_EARLY_RETURN(Mask.isValid() == false, "ERROR: Surface 2 Cloud Mapper: Mask Image is not Valid", false);

	std::vector<Eigen::Vector2f> UVs;
	std::vector<Eigen::Vector3f> Normals;
	std::vector<float> Dists;
	std::vector<core::SVertex> Samples;
	Eigen::Matrix<core::SVertex, -1, -1> VertexData;
	m_pSurface->dumpLatestLayer(VertexData);
	_HIVE_EARLY_RETURN(VertexData.size() == 0, "ERROR: Surface 2 Cloud Mapper: VertexData is Empty", false);

	/* Generate UV */
	core::CSurfaceUVGenerator Generator;
	if (Generator.generateUVSamples(Mask, vSPP) == false) return false;
	Generator.dumpSamples(UVs);
	_HIVE_EARLY_RETURN(UVs.size() == 0, "ERROR: Surface 2 Cloud Mapper:UV Vector is Empty", false);

	/* Sample Vertex */
	core::CVertexSampler VertexSampler;
	if (VertexSampler.setSurface(m_pSurface) == false) return false;
	if (VertexSampler.sample(UVs, Samples) == false) return false;

	/* Sample Normal */
	core::CNormalSampler NormalSampler;
	if (NormalSampler.setData(VertexData) == false) return false;
	for (const auto& e : UVs)
	{
		auto r = NormalSampler.sample(e);
		_HIVE_EARLY_RETURN(r.has_value() == false, "ERROR: Surface 2 Cloud Mapper: Normal Sampler Failed", false);
		Normals.emplace_back(r.value());
	}

	/* Sample Dist */
	core::CDistSampler DistSampler;
	if (DistSampler.setResource(vInpainted) == false) return false;
	if (DistSampler.samplebyUV(UVs, Dists) == false) return false;

	_HIVE_EARLY_RETURN(Normals.size() != Dists.size() || Dists.size() != Samples.size(), "ERROR: Surface 2 Cloud Mapper: Normals, Dists, UVs Size is not Same", false);
	hiveEventLogger::hiveOutputEvent("Start Mapping: ");

	/* Map */
	core::CSurface2PCMapper Mapper;
	for (int i = 0; i < Normals.size(); i++)
	{
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR8("Point [%1%]: Start Point(%2%, %3%, %4%), Normal(%5%, %6%, %7%), Dist %8%", i, Samples[i].x, Samples[i].y, Samples[i].z, Normals[i][0], Normals[i][1], Normals[i][2], Dists[i]));

		{
			if (Samples[i].u > 0.5f)
				Normals[i] *= -1;
		}

		auto r = Mapper.generatePoint(Samples[i], Normals[i], Dists[i]);

		_HIVE_EARLY_RETURN(r.has_value() == false, "ERROR: Surface 2 Cloud Mapper: Mapper generate Point Failed", false);
		m_NewCloud->push_back(r.value());
	}

	hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("New Cloud Size [%1%]", m_NewCloud->size()));

	return true;
}

bool CSurface2CloudMapper::__ValidCheck(const core::CHeightMap& vRaw, const core::CHeightMap& vInpainted, int vSPP)
{
	_HIVE_EARLY_RETURN(vRaw.isValid() == false, "ERROR: Surface 2 Cloud Mapper: Raw Image is not Valid", false);
	_HIVE_EARLY_RETURN(vInpainted.isValid() == false, "ERROR: Surface 2 Cloud Mapper: Inpainted Image is not Valid", false);
	_HIVE_EARLY_RETURN(vSPP < 1, "ERROR: Surface 2 Cloud Mapper: SPP <= 1", false);
	_HIVE_EARLY_RETURN(m_pSurface->IsPreComputed() == false, "ERROR: Surface 2 Cloud Mapper: Surface has not Pre Computed", false);
	return true;
}

