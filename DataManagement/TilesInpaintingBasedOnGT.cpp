#include "pch.h"
#include "TilesInpaintingBasedOnGT.h"
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

CTilesInpaintingBasedOnGT::CTilesInpaintingBasedOnGT()
	: m_pCloud(new PC_t)
{}

bool CTilesInpaintingBasedOnGT::setTilesInfo(float vRate, int vSizeX, int vSizeY)
{
	_HIVE_EARLY_RETURN(vRate < 0 || vRate > 1 || vSizeX <= 0 || vSizeY <= 0, _FORMAT_STR3("Para invalid, Rate [%1%], SizeX [%2%], SizeY [%3%]", vRate, vSizeX, vSizeY), false);
	m_TilesInfo._Rate = vRate;
	m_TilesInfo._SizeX = vSizeX;
	m_TilesInfo._SizeY = vSizeY;
	return true;
}

bool CTilesInpaintingBasedOnGT::setNurbsInfo(int vDegree, int vRefinement, int vIteration, int vSubNumber, int vSubLayer)
{
	_HIVE_EARLY_RETURN(vDegree <= 0 || vRefinement < 0 || vIteration < 0 || vSubNumber <= 0 || vSubLayer <= 0, _FORMAT_STR5("Para invalid, Degree [%1%], Refinement [%2%], Iteration [%3%], SubNumber [%4%], SubLayer [%5%]", vDegree, vRefinement, vIteration, vSubNumber, vSubLayer), false);
	m_NurbsInfo._Degree = vDegree;
	m_NurbsInfo._Iteration = vIteration;
	m_NurbsInfo._Refinement = vRefinement;
	m_NurbsInfo._SubNumber = vSubNumber;
	m_NurbsInfo._SubLayer = vSubLayer;
	return true;
}

bool CTilesInpaintingBasedOnGT::setMapInfo(int vWidth, int vHeight, int vSPP)
{
	_HIVE_EARLY_RETURN(vWidth <= 0 || vHeight <= 0 || vSPP <= 0, _FORMAT_STR3("Para invalid, Width [%1%], Height [%2%], SPP [%3%]", vWidth, vHeight, vSPP), false);
	m_MapInfo._Width = vWidth;
	m_MapInfo._Height = vHeight;
	m_MapInfo._SPP = vSPP;
	return true;
}

bool CTilesInpaintingBasedOnGT::run(const PC_t::Ptr& vRaw, const PC_t::Ptr& vSub, const PC_t::Ptr& vGT)
{
	_HIVE_EARLY_RETURN(vRaw == nullptr || vSub == nullptr || vGT == nullptr, "Cloud is nullptr", false);
	_HIVE_EARLY_RETURN(vRaw->size() == 0 || vSub->size() == 0 || vGT->size() == 0, "Cloud is empty", false);

	/* Set up Tiles */
	Eigen::Matrix<PC_t::Ptr, -1, -1> TilesRaw, TilesSub, TilesGT;
	_HIVE_EARLY_RETURN(__setupTiles(vRaw, TilesRaw) == false, "Raw tiles set up failed", false);
	_HIVE_EARLY_RETURN(__setupTiles(vSub, TilesSub) == false, "Raw tiles set up failed", false);
	_HIVE_EARLY_RETURN(__setupTiles(vGT, TilesGT) == false, "Raw tiles set up failed", false);
	
	/* Inpaint Tiles */
	Eigen::Matrix<PC_t::Ptr, -1, -1> CloudCulled;
	CloudCulled.resize(TilesRaw.rows(), TilesRaw.cols());

	std::vector<std::pair<int, int>> TileIds;
	for (int i = 0; i < TilesRaw.rows(); i++)
		for (int k = 0; k < TilesRaw.cols(); k++)
			TileIds.emplace_back(std::make_pair(i, k));

	std::mutex Mutex;

#pragma omp parallel for num_threads(TileIds.size())
	for (int Id = 0; Id < TileIds.size(); Id++)
	{
		int i = TileIds[Id].first;
		int k = TileIds[Id].second;

		const auto& pLocalRaw = TilesRaw.coeff(i, k);
		const auto& pLocalSub = TilesSub.coeff(i, k);
		const auto& pLocalGT = TilesGT.coeff(i, k);
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR5("Start Fitting Tile [%1%, %2%], Size [%3%] [%4%] [%5%]", i, k, pLocalRaw->size(), pLocalSub->size(), pLocalGT->size()));

		/* Generate Nurbs */
		core::CNurbsFitting Fitting;
		std::shared_ptr<pcl::on_nurbs::FittingSurface> Fit;

		if (Fitting.run(pLocalSub, m_NurbsInfo._Degree, m_NurbsInfo._Refinement, m_NurbsInfo._Iteration) == false)
		{
			hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Tile [%1%, %2%] fit failed", i, k));
			continue;
		}
		Fitting.dumpFitting(Fit);

		/* Organize Ctrlpts */
		Eigen::Matrix<core::SPoint, -1, -1> Ctrlpts;
		if (__extractCtrlpts(Fit->m_nurbs, Ctrlpts) == false)
		{
			hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Tile [%1%, %2%] extract ctrlpts failed", i, k));
			continue;
		}

		/* Generate Surface Mesh */
		std::string MeshSavePath = "Mesh/" + std::to_string(i) + "_" + std::to_string(k) + ".obj";
		core::CMultilayerSurface* pSurface(new core::CMultilayerSurface(m_NurbsInfo._Degree));
		pSurface->setControlPoints(Ctrlpts);
		pSurface->setIsSaveMesh(true, MeshSavePath);
		pSurface->setSubNumber(m_NurbsInfo._SubNumber);
		pSurface->setSubLayer(m_NurbsInfo._SubLayer);
		if (pSurface->preCompute() == false)
		{
			hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Tile [%1%, %2%] generate mesh failed", i, k));
			continue;
		}

		/* Project Point 2 Nurbs */
		std::vector<std::pair<float, Eigen::Vector2f>> DataRaw, DataGT;
		__projPoints(Fit, pSurface, pLocalRaw, DataRaw);
		__projPoints(Fit, pSurface, pLocalGT, DataGT);

		/* Generate Height Map */
		core::CHeightMapGenerator MapGeneraterRaw, MapGeneraterGT;
		core::CHeightMap Map, Mask, MapGT, MaskGT, Inpainted;
		if (MapGeneraterRaw.generateBySurface(DataRaw, m_MapInfo._Width, m_MapInfo._Height) == false || MapGeneraterGT.generateBySurface(DataGT, m_MapInfo._Width, m_MapInfo._Height) == false)
		{
			hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Tile [%1%, %2%] generate height map failed", i, k));
			continue;
		}
		MapGeneraterRaw.dumpHeightMap(Map);
		MapGeneraterGT.dumpHeightMap(MapGT);
		Map.generateMask(Mask);
		MapGT.generateMask(MaskGT);

		__saveImage(Mask, "Mask/Mask_" + std::to_string(i) + "_" + std::to_string(k) + ".png", 255);
		__saveImage(MaskGT, "Mask/InpaintedMask_" + std::to_string(i) + "_" + std::to_string(k) + ".png", 255);
		__saveImage(Map, "HeightMap/" + std::to_string(i) + "_" + std::to_string(k) + ".png", 100);
		__saveImage(MapGT, "HeightMap/GT_" + std::to_string(i) + "_" + std::to_string(k) + ".png", 100);
		__tuneMapBoundary(MaskGT, Mask);

		if (__isMaskNoHole(Mask))
		{
			PC_t::Ptr pTemp(new PC_t);
			CloudCulled.coeffRef(i, k) = pTemp;
			continue;
		}

		/* Image Inpainting */
		/*CImageInpainting Inpainter;
		Inpainter.run(Map, Inpainted);
		__saveImage(Inpainted, "HeightMap/Inpainted_" + std::to_string(i) + "_" + std::to_string(k) + ".png", 100);*/

		/* Map 2 Cloud */
		std::shared_ptr<core::CMultilayerSurface> pTrSurface(pSurface);
		CNurbs2CloudMapper Mapper;

		if (Mapper.run(Mask, MapGT, m_MapInfo._SPP, Fit, pTrSurface) == false) continue;
		PC_t::Ptr pNew(new PC_t);
		Mapper.dumpCloud(pNew);

		/* Cull */
		core::CAABBEstimation Estimation(pLocalRaw);
		const auto Box = Estimation.compute();

		if (Box.isValid() == false) continue;
		float ThresX = (Box._Max[0] - Box._Min[0]) * m_TilesInfo._Rate * 0.7f;
		float ThresY = (Box._Max[1] - Box._Min[1]) * m_TilesInfo._Rate * 0.7f;

		PC_t::Ptr pCulled(new PC_t);
		for (const auto& e : *pNew)
		{
			if (e.x < Box._Min.x() || e.x > Box._Max.x() || e.y < Box._Min.y() || e.y > Box._Max.y()) continue;
			if (std::fabsf(e.x - Box._Min.x()) < ThresX || std::fabsf(e.x - Box._Max.x()) < ThresX || std::fabsf(e.y - Box._Min.y()) < ThresY || std::fabsf(e.y - Box._Max.y()) < ThresY) continue;

			pCulled->emplace_back(e);
		}

		Mutex.lock();
		CloudCulled.coeffRef(i, k) = pCulled;
		Mutex.unlock();
	}

	for (int i = 0; i < CloudCulled.rows(); i++)
		for (int k = 0; k < CloudCulled.cols(); k++)
		{
			hiveEventLogger::hiveOutputEvent(_FORMAT_STR3("Tile [%1%, %2%], Size [%3%]", i, k, CloudCulled.coeff(i, k)->size()));
			for (const auto e : *CloudCulled.coeff(i, k))
				m_pCloud->emplace_back(Point_t(e.x, e.y, e.z, (std::uint8_t)253, (std::uint8_t)199, (std::uint8_t)83, (std::uint8_t)255));
		}
}

bool CTilesInpaintingBasedOnGT::__setupTiles(const PC_t::Ptr& vCloud, Eigen::Matrix<PC_t::Ptr, -1, -1>& voTiles)
{
	core::CTiler Tiler;
	std::vector<std::vector<int>> Tiles;
	std::vector<std::pair<int, int>> Coors;
	_HIVE_EARLY_RETURN(Tiler.run(vCloud, m_TilesInfo._SizeX, m_TilesInfo._SizeY, m_TilesInfo._Rate) == false, "Tiler failed", false);
	Tiler.dumpResult(Tiles, Coors);

	voTiles.resize(m_TilesInfo._SizeX, m_TilesInfo._SizeY);
	for (int i = 0; i < Tiles.size(); i++)
	{
		const auto& t = Tiles[i];
		PC_t::Ptr pLocal(new PC_t);
		for (auto e : t)
			pLocal->emplace_back(vCloud->at(e));

		int Row = Coors[i].first;
		int Col = Coors[i].second;

		_HIVE_EARLY_RETURN(Row < 0 || Row >= voTiles.rows() || Col < 0 || Col >= voTiles.cols(), _FORMAT_STR3("Coor %1% [%2%, %3%] invalid", i, Row, Col), false);
		voTiles.coeffRef(Row, Col) = pLocal;
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR3("Raw Tile [%1%, %2%], Size [%3%]", Row, Col, pLocal->size()));
	}
	return true;
}

bool CTilesInpaintingBasedOnGT::__extractCtrlpts(const ON_NurbsSurface& vNurbs, Eigen::Matrix<core::SPoint, -1, -1>& voCtrlpts)
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

int CountMin = 0; int CountMax = 0;

void CTilesInpaintingBasedOnGT::__projPoints(const std::shared_ptr<pcl::on_nurbs::FittingSurface>& vFit, core::CMultilayerSurface* vSurface, const PC_t::Ptr& vCloud, std::vector<std::pair<float, Eigen::Vector2f>>& voData)
{
	/* Sample Normal */
	Eigen::Matrix<core::SVertex, -1, -1> VertexData;
	vSurface->dumpLatestLayer(VertexData);
	core::CNormalSampler NormalSampler;
	_HIVE_EARLY_RETURN(NormalSampler.setData(VertexData) == false, "ERROR: VertexData is empty",);

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
			//hiveEventLogger::hiveOutputEvent(_FORMAT_STR4("Point Number [%1%], Pos [%2%, %3%, %4%] calc proj info failed", i, PCLPoint.x, PCLPoint.y, PCLPoint.z));
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
		/*Domain[0] = (Domain[0] > Dist) ? Domain[0] : Dist;*/
		/*Domain[1] = (Domain[1] < Dist) ? Domain[1] : Dist;*/
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
	{
		PC_t::Ptr pCloud(new PC_t);
		pCloud->emplace_back(MinPair.first);
		pCloud->emplace_back(MinPair.second);
		pcl::io::savePLYFileBinary("MinPair/" + std::to_string(CountMin++) + ".ply", *pCloud);
	}
	{
		PC_t::Ptr pCloud(new PC_t);
		pCloud->emplace_back(MaxPair.first);
		pCloud->emplace_back(MaxPair.second);
		pcl::io::savePLYFileBinary("MaxPair/" + std::to_string(CountMax++) + ".ply", *pCloud);
	}
}

void CTilesInpaintingBasedOnGT::__tuneMapBoundary(const core::CHeightMap& vGT, core::CHeightMap& vioRaw)
{
	/* Expand hole area */
	const core::CHeightMap Copy = vioRaw;
	for (int i = 1; i < vioRaw.getWidth() - 1; i++)
		for (int k = 1; k < vioRaw.getHeight() - 1; k++)
			if (Copy.getValueAt(i - 1, k) == 1 || Copy.getValueAt(i, k - 1) == 1 || Copy.getValueAt(i + 1, k) == 1 || Copy.getValueAt(i, k + 1) == 1)
				vioRaw.setValueAt(1, i, k);

	for (int i = 1; i < vioRaw.getWidth() - 1; i++)
		for (int k = 1; k < vioRaw.getHeight() - 1; k++)
			if (Copy.getValueAt(i, k) == 1 && vGT.getValueAt(i, k) == 1)
				vioRaw.setValueAt(0, i, k);
}

bool CTilesInpaintingBasedOnGT::__isMaskNoHole(const core::CHeightMap& vMask)
{
	_HIVE_EARLY_RETURN(vMask.isValid() == false, "Map is invalid", false);

	for (int i = 0; i < vMask.getWidth(); i++)
		for (int k = 0; k < vMask.getHeight(); k++)
		{
			if (vMask.getValueAt(i, k) == 1)
				return false;
		}

	return true;
}

void CTilesInpaintingBasedOnGT::__saveImage(const core::CHeightMap& vMap, const std::string& vPath, int vCoeff)
{
	Eigen::MatrixXf ImageData;
	float Max = -FLT_MAX;
	ImageData.resize(vMap.getWidth(), vMap.getHeight());
	for (int i = 0; i < ImageData.rows(); i++)
		for (int k = 0; k < ImageData.cols(); k++)
		{
			Max = (Max > vMap.getValueAt(i, k) * vCoeff) ? Max : vMap.getValueAt(i, k) * vCoeff;
			ImageData.coeffRef(i, k) = vMap.getValueAt(i, k) * vCoeff;
		}
	hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("Max: [%1%]", Max));
	common::saveImage(ImageData.cast<int>(), vPath);
}

