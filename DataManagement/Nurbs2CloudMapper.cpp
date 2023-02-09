#include "pch.h"
#include "Nurbs2CloudMapper.h"

#include "NurbsFitting.h"
#include "SurfaceUVGenerator.h"
#include "NormalSampler.h"
#include "DistSampler.h"
#include "Surface2PCMapper.h"

using namespace dataManagement;

CNurbs2CloudMapper::CNurbs2CloudMapper()
	: m_pCloud(new PC_t)
{}

bool CNurbs2CloudMapper::run(const core::CHeightMap& vMask, const core::CHeightMap& vInpainted, int vSPP, const std::shared_ptr<pcl::on_nurbs::FittingSurface>& vFit, const std::shared_ptr<core::CMultilayerSurface>& vSurface)
{
	_HIVE_EARLY_RETURN(vMask.isValid() == false, "ERROR: Mask Image is not Valid", false);

	std::vector<Eigen::Vector2f> UVs;
	std::vector<Eigen::Vector3f> Normals;
	std::vector<float> Dists;
	std::vector<core::SVertex> Samples;
	Eigen::Matrix<core::SVertex, -1, -1> VertexData;
	vSurface->dumpLatestLayer(VertexData);
	hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("dump Vertex Data [%1% x %2%] dims from Refined Surface", VertexData.rows(), VertexData.cols()));
	_HIVE_EARLY_RETURN(VertexData.size() == 0, "ERROR: VertexData is Empty", false);

	/* Generate UV */
	core::CSurfaceUVGenerator Generator;
	if (Generator.generateUVSamples(vMask, vSPP) == false) return false;
	Generator.dumpSamples(UVs);
	_HIVE_EARLY_RETURN(UVs.size() == 0, "ERROR: UV Vector is Empty", false);

	/* Sample Vertex */
	for (const auto& e : UVs)
	{
		const auto Sample = vFit->m_nurbs.PointAt(e[0], e[1]);
		if (std::isnan(Sample.x) || std::isnan(Sample.y) || std::isnan(Sample.z))
			hiveEventLogger::hiveOutputEvent(_FORMAT_STR5("nan: UV: [%1%, %2%], Value: [%3%, %4%, %5%]", e[0], e[1], Sample.x, Sample.y, Sample.z));
		Samples.emplace_back(core::SVertex(Sample.x, Sample.y, Sample.z));
	}
	hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("Sample Vector size: %1%", Samples.size()));

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
			/*if (Samples[i].u < 0.5f)
				Normals[i] *= -1;*/
		}

		//if (std::isnan(Dists[i]) || std::fabsf(Dists[i]) > 100) continue;
		Dists[i] = 0;

		auto r = Mapper.generatePoint(Samples[i], Normals[i], Dists[i]);

		_HIVE_EARLY_RETURN(r.has_value() == false, "ERROR: Surface 2 Cloud Mapper: Mapper generate Point Failed", false);
		m_pCloud->emplace_back(r.value());
	}

	hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("New Cloud Size [%1%]", m_pCloud->size()));

	return true;
}
