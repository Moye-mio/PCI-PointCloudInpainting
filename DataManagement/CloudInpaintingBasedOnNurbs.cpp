#include "pch.h"
#include "CloudInpaintingBasedOnNurbs.h"

#include "GAInterface.h"
#include "NurbsFitting.h"
#include "HeightMapGenerator.h"
#include "ImageInpainting.h"
#include "SurfaceUVGenerator.h"
#include "NormalSampler.h"
#include "DistSampler.h"
#include "Surface2PCMapper.h"

using namespace dataManagement;

CCloudInpaintingBasedOnNurbs::CCloudInpaintingBasedOnNurbs()
	: m_pCloud(new PC_t)
{}

bool CCloudInpaintingBasedOnNurbs::run(const PC_t::Ptr& vCloud)
{
	_HIVE_EARLY_RETURN(vCloud == nullptr, "Cloud is nullptr", false);
	_HIVE_EARLY_RETURN(vCloud->size() == 0, "Cloud is empty", false);

	/* GA */
	/*std::vector<std::vector<int>> Clusters;
	GA::run(vCloud, Clusters);*/

	/* Generate Nurbs */
	int Degree = 3;
	int Refinement = 5;
	int Iteration = 10;
	core::CNurbsFitting Fitting;
	std::shared_ptr<pcl::on_nurbs::FittingSurface> Fit;
	_HIVE_EARLY_RETURN(Fitting.run(vCloud, Degree, Refinement, Iteration) == false, "Nurbs Fitting failed", false);
	Fitting.dumpFitting(Fit);

	/* Organize Ctrlpts */
	Eigen::Matrix<core::SPoint, -1, -1> Ctrlpts;
	_HIVE_EARLY_RETURN(__extractCtrlpts(Fit->m_nurbs, Ctrlpts) == false, "Extract Ctrlpts failed", false);

	/* Generate Surface Mesh */
	int SubNumber = 2;
	int SubLayer = 2;
	core::CMultilayerSurface* pSurface(new core::CMultilayerSurface(Degree));
	pSurface->setControlPoints(Ctrlpts);
	pSurface->setIsSaveMesh(true, "Surface.obj");
	pSurface->setSubNumber(SubNumber);
	pSurface->setSubLayer(SubLayer);
	_HIVE_EARLY_RETURN(pSurface->preCompute() == false, "Surface Mesh precompute failed", false);

	/* Project Point 2 Nurbs */
	std::vector<std::pair<float, Eigen::Vector2f>> Data;
	__projPoints(Fit, pSurface, vCloud, Data);

	/* Generate Height Map */
	core::CHeightMapGenerator MapGenerater;
	core::CHeightMap Map, Mask, Inpainted, InpaintedMask;
	int Width = 32;
	int Height = 32;
	_HIVE_EARLY_RETURN(MapGenerater.generateBySurface(Data, Width, Height) == false, "Generate Height Map failed", false);
	MapGenerater.dumpHeightMap(Map);
	Map.generateMask(Mask);
	__saveImage(Map, "Map.png", 1000);
	__saveImage(Mask, "Mask.png", 255);
	__tuneMapBoundary(Mask);
	//__saveImage(Mask, "NewMask.png", 255);
	
	/* Image Inpainting */
	CImageInpainting Inpainter;
	Inpainter.run(Map, Inpainted);
	__saveImage(Inpainted, "Inpainted.png", 1000);
	Inpainted.generateMask(InpaintedMask);
	__saveImage(InpaintedMask, "InpaintedMask.png", 1);
	
	/* Map 2 Cloud */
	int SPP = 10;
	PC_t::Ptr pCloud(new PC_t);
	std::shared_ptr<core::CMultilayerSurface> pTrSurface(pSurface);
	_HIVE_EARLY_RETURN(__map2Cloud(Mask, Inpainted, SPP, Fit, pTrSurface, pCloud) == false, "Map 2 Cloud failed", false);

	core::CAABBEstimation Estimation(vCloud);
	const auto Box = Estimation.compute();
	_HIVE_EARLY_RETURN(Box.isValid() == false, "AABB is invalid", false);
	
	for (const auto& e : *pCloud)
	{
		if (e.x < Box._Min.x() || e.y < Box._Min.y() || e.x > Box._Max.x() || e.y > Box._Max.y())
			continue;
		m_pCloud->emplace_back(e);
	}

	return true;
}

bool CCloudInpaintingBasedOnNurbs::__extractCtrlpts(const ON_NurbsSurface& vNurbs, Eigen::Matrix<core::SPoint, -1, -1>& voCtrlpts)
{
	voCtrlpts.resize(vNurbs.m_cv_count[0], vNurbs.m_cv_count[1]);
	for (int i = 0; i < voCtrlpts.rows(); i++)
		for (int k = 0; k < voCtrlpts.cols(); k++)
		{
			ON_3dPoint p;
			vNurbs.GetCV(i, k, p);
			_HIVE_EARLY_RETURN(std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z), _FORMAT_STR5("Ctrlpts [%1%, %2%]: [%3%, %4%, %5%]", i, k, p.x, p.y, p.z), false);
			voCtrlpts.coeffRef(i, k) = core::SPoint(p.x, p.y, p.z);
		}

	hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("extract Ctrlpts successfully [%1%]", voCtrlpts.size()));
	return true;
}

void CCloudInpaintingBasedOnNurbs::__projPoints(const std::shared_ptr<pcl::on_nurbs::FittingSurface>& vFit, core::CMultilayerSurface* vSurface, const PC_t::Ptr& vCloud, std::vector<std::pair<float, Eigen::Vector2f>>& voData)
{
	for (int i = 0; i < vCloud->size(); i++)
	{
		const auto& PCLPoint = vCloud->at(i);
		auto r = vSurface->calcProj(core::SPoint(PCLPoint.x, PCLPoint.y, PCLPoint.z));
		if (r.has_value() == false)
		{
			hiveEventLogger::hiveOutputEvent(_FORMAT_STR4("Point Number [%1%], Pos [%2%, %3%, %4%] calc proj info failed", i, PCLPoint.x, PCLPoint.y, PCLPoint.z));
			continue;
		}
		Eigen::Vector2d Hint((double)r->_UV[0], (double)r->_UV[1]);

		Eigen::Vector3d Point(vCloud->at(i).x, vCloud->at(i).y, vCloud->at(i).z);
		Eigen::Vector3d p;
		Eigen::Vector2d UV = vFit->inverseMapping(vFit->m_nurbs, Point, Hint, p, 100, 1e-6, true);
		if (UV[0] < 0 || UV[0] > 1 || UV[1] < 0 || UV[1] > 1 || std::isnan(p.x()) || std::isnan(p.y()) || std::isnan(p.z()))
		{
			hiveEventLogger::hiveOutputEvent(_FORMAT_STR6("Point Number [%1%], UV [%2%, %3%], Pos [%4%, %5%, %6%]", i, UV[0], UV[1], p[0], p[1], p[2]));
			continue;
		}
		const auto Sample = vFit->m_nurbs.PointAt(UV[0], UV[1]);

		p = Eigen::Vector3d(Sample.x, Sample.y, Sample.z);
		float Dist = (Point - p).norm();
		voData.emplace_back(std::make_pair(Dist, UV.cast<float>()));
		/*hiveEventLogger::hiveOutputEvent(_FORMAT_STR8("[%1%, %2%, %3%] -> [%4%, %5%, %6%], UV: [%7%, %8%]", Point[0], Point[1], Point[2], p[0], p[1], p[2], UV[0], UV[1]));
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("Dist: [%1%]", Dist));*/
	}
	hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Point Cloud [%1%] convert 2 Data [%2%]", vCloud->size(), voData.size()));
}

void CCloudInpaintingBasedOnNurbs::__tuneMapBoundary(core::CHeightMap& vioMask)
{
	//int TuneScale = 3;
	//for (int i = 0; i < vioMask.getWidth(); i++)
	//	for (int k = 0; k < vioMask.getHeight(); k++)
	//	{
	//		if (i < TuneScale || k < TuneScale || i > vioMask.getWidth() - TuneScale - 1 || k > vioMask.getHeight() - TuneScale - 1)
	//		{
	//			vioMask.setValueAt(0, i, k);
	//		}
	//	}

	const core::CHeightMap Map = vioMask;

	for (int i = 1; i < vioMask.getWidth() - 1; i++)
		for (int k = 1; k < vioMask.getHeight() - 1; k++)
		{
			if (Map.getValueAt(i - 1, k) == 1 || Map.getValueAt(i, k - 1) == 1 || Map.getValueAt(i + 1, k) == 1 || Map.getValueAt(i, k + 1) == 1)
				vioMask.setValueAt(1, i, k);
		}
}

bool CCloudInpaintingBasedOnNurbs::__map2Cloud(const core::CHeightMap& vMask, const core::CHeightMap& vInpainted, int vSPP, const std::shared_ptr<pcl::on_nurbs::FittingSurface>& vFit, const std::shared_ptr<core::CMultilayerSurface>& vSurface, PC_t::Ptr& vCloud)
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
		vCloud->emplace_back(r.value());
	}

	hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("New Cloud Size [%1%]", vCloud->size()));

	return true;
}

void CCloudInpaintingBasedOnNurbs::__saveImage(const core::CHeightMap& vMap, const std::string& vPath, int vCoeff)
{
	Eigen::MatrixXf ImageData;
	ImageData.resize(vMap.getWidth(), vMap.getHeight());
	for (int i = 0; i < ImageData.rows(); i++)
		for (int k = 0; k < ImageData.cols(); k++)
			ImageData.coeffRef(i, k) = vMap.getValueAt(i, k) * vCoeff;

	common::saveImage(ImageData.cast<int>(), vPath);
}
