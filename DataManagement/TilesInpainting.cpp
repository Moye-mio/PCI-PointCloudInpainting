#include "pch.h"
#include "TilesInpainting.h"
#include "Tiler.h"
#include "Nurbs2CloudMapper.h"
#include "NurbsFitting.h"
#include "HeightMapGenerator.h"
#include "ImageInpainting.h"
#include "SurfaceUVGenerator.h"
#include "NormalSampler.h"
#include "DistSampler.h"
#include "Surface2PCMapper.h"

using namespace dataManagement;

bool CTilesInpainting::run(const PC_t::Ptr& vRaw, const PC_t::Ptr& vSub, int vSizeX, int vSizeY, float vRate)
{
	_HIVE_EARLY_RETURN(vRaw == nullptr || vSub == nullptr, "Cloud is nullptr", false);
	_HIVE_EARLY_RETURN(vRaw->size() == 0 || vSub->size() == 0, "Cloud is empty", false);
	_HIVE_EARLY_RETURN(vSizeX <= 0 || vSizeY <= 0, "Size is Invalid", false);
	_HIVE_EARLY_RETURN(vRate < 0 || vRate > 1, "Rate is Invalid", false);

	std::vector<std::vector<int>> Tiles1, Tiles2;
	std::vector<std::pair<int, int>> Coors;
	core::CTiler Tiler1, Tiler2;
	_HIVE_EARLY_RETURN(Tiler1.run(vRaw, vSizeX, vSizeY, vRate) == false, "Tiler failed", false);
	Tiler1.dumpResult(Tiles1, Coors);
	_HIVE_EARLY_RETURN(Tiles1.size() == 0 || Tiles1.size() != Coors.size(), "Tiles and Coors size error", false);

	_HIVE_EARLY_RETURN(Tiler2.run(vSub, vSizeX, vSizeY, vRate) == false, "Tiler failed", false);
	Tiler2.dumpResult(Tiles2, Coors);
	_HIVE_EARLY_RETURN(Tiles2.size() == 0 || Tiles2.size() != Coors.size(), "Tiles and Coors size error", false);

	Eigen::Matrix<PC_t::Ptr, -1, -1> RawTiles, SubTiles;
	RawTiles.resize(vSizeX, vSizeY);
	SubTiles.resize(vSizeX, vSizeY);

	for (int i = 0; i < Tiles1.size(); i++)
	{
		const auto& t = Tiles1[i];
		PC_t::Ptr pLocal(new PC_t);
		for (auto e : t)
			pLocal->emplace_back(vRaw->at(e));

		int Row = Coors[i].first;
		int Col = Coors[i].second;

		_HIVE_EARLY_RETURN(Row < 0 || Row >= RawTiles.rows() || Col < 0 || Col >= RawTiles.cols(), _FORMAT_STR3("Coor %1% [%2%, %3%] invalid", i, Row, Col), false);
		RawTiles.coeffRef(Row, Col) = pLocal;
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR3("Raw Tile [%1%, %2%], Size [%3%]", Row, Col, pLocal->size()));
	}

	for (int i = 0; i < Tiles2.size(); i++)
	{
		const auto& t = Tiles2[i];
		PC_t::Ptr pLocal(new PC_t);
		for (auto e : t)
			pLocal->emplace_back(vSub->at(e));

		int Row = Coors[i].first;
		int Col = Coors[i].second;

		_HIVE_EARLY_RETURN(Row < 0 || Row >= SubTiles.rows() || Col < 0 || Col >= SubTiles.cols(), _FORMAT_STR3("Coor %1% [%2%, %3%] invalid", i, Row, Col), false);
		SubTiles.coeffRef(Row, Col) = pLocal;
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR3("Sub Tile [%1%, %2%], Size [%3%]", Row, Col, pLocal->size()));
	}

	Eigen::Matrix<PC_t::Ptr, -1, -1> CloudCulled;
	CloudCulled.resize(vSizeX, vSizeY);

	int ThreadSize = 16;
	std::mutex Mutex;

	std::vector<std::pair<int, int>> TileIds;
	for (int i = 0; i < RawTiles.rows(); i++)
		for (int k = 0; k < RawTiles.cols(); k++)
			TileIds.emplace_back(std::make_pair(i, k));

#pragma omp parallel for num_threads(TileIds.size())
	for (int Id = 0; Id < TileIds.size(); Id++)
	{
		int i = TileIds[Id].first;
		int k = TileIds[Id].second;

		const auto& pLocal = RawTiles.coeff(i, k);
		const auto& pSubLocal = SubTiles.coeff(i, k);

		hiveEventLogger::hiveOutputEvent(_FORMAT_STR4("Start Fitting Tile [%1%, %2%], Size [%3%] [%4%]", i, k, pLocal->size(), pSubLocal->size()));

		/* Generate Nurbs */
		int Degree = 3;
		int Refinement = 4;
		int Iteration = 10;
		core::CNurbsFitting Fitting;
		std::shared_ptr<pcl::on_nurbs::FittingSurface> Fit;

		if (Fitting.run(pSubLocal, Degree, Refinement, Iteration) == false) continue;
		//_HIVE_EARLY_RETURN(Fitting.run(pSubLocal, Degree, Refinement, Iteration) == false, "Nurbs Fitting failed", false);
		Fitting.dumpFitting(Fit);

		/* Organize Ctrlpts */
		Eigen::Matrix<core::SPoint, -1, -1> Ctrlpts;

		if (__extractCtrlpts(Fit->m_nurbs, Ctrlpts) == false) continue;
		//_HIVE_EARLY_RETURN(__extractCtrlpts(Fit->m_nurbs, Ctrlpts) == false, "Extract Ctrlpts failed", false);

		/* Generate Surface Mesh */
		int SubNumber = 2;
		int SubLayer = 2;
		std::string MeshSavePath = std::to_string(i) + "_" + std::to_string(k) + ".obj";
		core::CMultilayerSurface* pSurface(new core::CMultilayerSurface(Degree));
		pSurface->setControlPoints(Ctrlpts);
		pSurface->setIsSaveMesh(true, MeshSavePath);
		pSurface->setSubNumber(SubNumber);
		pSurface->setSubLayer(SubLayer);

		if (pSurface->preCompute() == false) continue;
		//_HIVE_EARLY_RETURN(pSurface->preCompute() == false, "Surface Mesh precompute failed", false);

		/* Project Point 2 Nurbs */
		std::vector<std::pair<float, Eigen::Vector2f>> Data;
		__projPoints(Fit, pSurface, pLocal, Data);

		/* Generate Height Map */
		core::CHeightMapGenerator MapGenerater;
		core::CHeightMap Map, Mask, Inpainted, InpaintedMask;
		int Width = 128;
		int Height = 128;

		if (MapGenerater.generateBySurface(Data, Width, Height) == false) continue;
		//_HIVE_EARLY_RETURN(MapGenerater.generateBySurface(Data, Width, Height) == false, "Generate Height Map failed", false);
		MapGenerater.dumpHeightMap(Map);
		Map.generateMask(Mask);
		__tuneMapBoundary(Mask);

		if (__isMapNoHole(Map))
		{
			PC_t::Ptr pTemp(new PC_t);
			CloudCulled.coeffRef(i, k) = pTemp;
			continue;
		}

		/* Image Inpainting */
		CImageInpainting Inpainter;
		Inpainter.run(Map, Inpainted);

		/* Map 2 Cloud */
		int SPP = 10;
		std::shared_ptr<core::CMultilayerSurface> pTrSurface(pSurface);
		CNurbs2CloudMapper Mapper;

		if (Mapper.run(Mask, Inpainted, SPP, Fit, pTrSurface) == false) continue;
		//_HIVE_EARLY_RETURN(Mapper.run(Mask, Inpainted, SPP, Fit, pTrSurface) == false, "Map 2 Cloud failed", false);
		PC_t::Ptr pMapper(new PC_t);
		Mapper.dumpCloud(pMapper);

		/* Cull */
		core::CAABBEstimation Estimation(pLocal);
		const auto Box = Estimation.compute();

		if (Box.isValid() == false) continue;
		//_HIVE_EARLY_RETURN(Box.isValid() == false, "AABB is invalid", false);
		float ThresX = (Box._Max[0] - Box._Min[0]) * vRate * 0.7f;
		float ThresY = (Box._Max[1] - Box._Min[1]) * vRate * 0.7f;

		PC_t::Ptr pCulled(new PC_t);
		for (const auto& e : *pMapper)
		{
			if (e.x < Box._Min.x() || e.x > Box._Max.x() || e.y < Box._Min.y() || e.y > Box._Max.y()) continue;
			if (std::fabsf(e.x - Box._Min.x()) < ThresX || std::fabsf(e.x - Box._Max.x()) < ThresX || std::fabsf(e.y - Box._Min.y()) < ThresY || std::fabsf(e.y - Box._Max.y()) < ThresY) continue;

			pCulled->emplace_back(e);
		}

		/* Give Color */
		for (auto& e : *pCulled)
		{
			e.r = 253;
			e.g = 199;
			e.b = 83;
		}

		Mutex.lock();
		CloudCulled.coeffRef(i, k) = pCulled;
		Mutex.unlock();

		/* Save Single Tile */
		bool IsSave = false;
		if (IsSave)
		{
			PC_t::Ptr pSave(new PC_t);
			for (const auto e : *pCulled)
				pSave->emplace_back(e);

			/* Merge */
			for (const auto& e : *pLocal)
				pSave->emplace_back(Point_t(e.x, e.y, e.z, (std::uint8_t)255, (std::uint8_t)255, (std::uint8_t)255, (std::uint8_t)255));

			std::string Path = std::to_string(i) + "_" + std::to_string(k) + ".ply";
			pcl::io::savePLYFileBinary(Path, *pSave);
			hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Save Cloud [%1%] Size [%2%]", Path, pSave->size()));
		}
	}

	PC_t::Ptr pTotal(new PC_t);
	for (int i = 0; i < CloudCulled.rows(); i++)
		for (int k = 0; k < CloudCulled.cols(); k++)
		{
			hiveEventLogger::hiveOutputEvent(_FORMAT_STR3("Add [%1%, %2%] to Total, Size [%3%]", i, k, CloudCulled.coeff(i, k)->size()));
			for (const auto& e : *CloudCulled.coeff(i, k))
				pTotal->emplace_back(e);
		}
		
	for (const auto& e : *vRaw)
		pTotal->emplace_back(Point_t(e.x, e.y, e.z, (std::uint8_t)255, (std::uint8_t)255, (std::uint8_t)255, (std::uint8_t)255));

	pcl::io::savePLYFileBinary("Total.ply", *pTotal);

	return true;
}

bool CTilesInpainting::__extractCtrlpts(const ON_NurbsSurface& vNurbs, Eigen::Matrix<core::SPoint, -1, -1>& voCtrlpts)
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

void CTilesInpainting::__projPoints(const std::shared_ptr<pcl::on_nurbs::FittingSurface>& vFit, core::CMultilayerSurface* vSurface, const PC_t::Ptr& vCloud, std::vector<std::pair<float, Eigen::Vector2f>>& voData)
{
	/* Sample Normal */
	Eigen::Matrix<core::SVertex, -1, -1> VertexData;
	vSurface->dumpLatestLayer(VertexData);
	core::CNormalSampler NormalSampler;
	_HIVE_EARLY_RETURN(NormalSampler.setData(VertexData) == false, "ERROR: VertexData is empty", );

	int CountPositive = 0;
	int CountNegative = 0;
	int Invalid = 0;
	Eigen::Vector2f Domain = { -FLT_MAX, FLT_MAX };
	std::pair<Point_t, Point_t> MinPair, MaxPair;
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

		Eigen::Vector3d Point(PCLPoint.x, PCLPoint.y, PCLPoint.z);
		Eigen::Vector3d p;
		Eigen::Vector2d UV = vFit->inverseMapping(vFit->m_nurbs, Point, Hint, p, 100, 1e-6, true);
		if (UV[0] < 0 || UV[0] > 1 || UV[1] < 0 || UV[1] > 1 || std::isnan(p.x()) || std::isnan(p.y()) || std::isnan(p.z()))
		{
			hiveEventLogger::hiveOutputEvent(_FORMAT_STR6("Point Number [%1%], UV [%2%, %3%], Pos [%4%, %5%, %6%]", i, UV[0], UV[1], p[0], p[1], p[2]));
			continue;
		}
		const auto Sample = vFit->m_nurbs.PointAt(UV[0], UV[1]);
		const auto Normal = NormalSampler.sample(UV.cast<float>());
		_HIVE_EARLY_RETURN(Normal.has_value() == false, _FORMAT_STR2("Normal Sampler Failed", UV[0], UV[1]), );

		p = Eigen::Vector3d(Sample.x, Sample.y, Sample.z);
		float Dist = (Point - p).norm();

		if ((Point - p).dot(Normal.value().cast<double>()) < 0)
			Dist *= -1;

		core::CAABBEstimation Estimation(vCloud);
		const auto Box = Estimation.compute();
		if (Box._Max[2] - Box._Min[2] < std::fabsf(Dist))
		{
			Invalid++;
			continue;
		}

		if (Dist >= 0)	CountPositive++;
		else			CountNegative++;
		if (Domain[0] < Dist)
		{
			Domain[0] = Dist;
			MaxPair = std::make_pair(PCLPoint, Point_t(p.x(), p.y(), p.z(), (std::uint8_t)255, (std::uint8_t)0, (std::uint8_t)0, (std::uint8_t)255));
		}

		if (Domain[1] > Dist)
		{
			Domain[1] = Dist;
			MinPair = std::make_pair(PCLPoint, Point_t(p.x(), p.y(), p.z(), (std::uint8_t)255, (std::uint8_t)0, (std::uint8_t)0, (std::uint8_t)255));
		}

		voData.emplace_back(std::make_pair(Dist, UV.cast<float>()));
	}
	hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Point Cloud [%1%] convert 2 Data [%2%]", vCloud->size(), voData.size()));
	hiveEventLogger::hiveOutputEvent(_FORMAT_STR5("Positive: [%1%]; Negative: [%2%]; Invalid [%3%]; Domain [%4%, %5%]", CountPositive, CountNegative, Invalid, Domain[0], Domain[1]));
}

void CTilesInpainting::__tuneMapBoundary(core::CHeightMap& vioMask)
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

bool CTilesInpainting::__isMapNoHole(const core::CHeightMap& vMap)
{
	_HIVE_EARLY_RETURN(vMap.isValid() == false, "Map is invalid", false);

	for (int i = 0; i < vMap.getWidth(); i++)
		for (int k = 0; k < vMap.getHeight(); k++)
		{
			if (vMap.isEmptyValue(i, k))
				return false;
		}

	return true;
}
