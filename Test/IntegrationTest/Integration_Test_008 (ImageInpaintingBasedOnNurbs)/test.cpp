#include "pch.h"

const std::string Path1 = TESTMODEL_DIR + std::string("/Trimmed/Scene/2.ply");
const std::string Path2 = TESTMODEL_DIR + std::string("/Trimmed/Scene/2_Sub.ply");

PC_t::Ptr load(const std::string& vPath)
{
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<dataManagement::IPCLoader>(hiveUtility::hiveGetFileSuffix(vPath));
	_HIVE_EARLY_RETURN(pTileLoader == nullptr, "TileLoader is nullptr", nullptr);
	PC_t::Ptr pCloud = pTileLoader->loadDataFromFile(vPath);
	_HIVE_EARLY_RETURN(pCloud->size() == 0, "Cloud is empty", nullptr);
	return pCloud;
}

bool extractCtrlpts(const ON_NurbsSurface& vNurbs, Eigen::Matrix<core::SPoint, -1, -1>& voCtrlpts)
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

void projPoints(const std::shared_ptr<pcl::on_nurbs::FittingSurface>& vFit, core::CMultilayerSurface* vSurface, const PC_t::Ptr& vCloud, std::vector<std::pair<float, Eigen::Vector2f>>& voData)
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
	}
	hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Point Cloud [%1%] convert 2 Data [%2%]", vCloud->size(), voData.size()));
}

std::pair<float, float> calcExtreme(const core::CHeightMap& vMap)
{
	std::pair<float, float> Dists(FLT_MAX, -FLT_MAX);
	for (int i = 0; i < vMap.getWidth(); i++)
		for (int k = 0; k < vMap.getHeight(); k++)
		{
			if (vMap.isEmptyValue(i, k)) continue;
			Dists.first = (vMap.getValueAt(i, k) < Dists.first) ? vMap.getValueAt(i, k) : Dists.first;
			Dists.second = (vMap.getValueAt(i, k) > Dists.second) ? vMap.getValueAt(i, k) : Dists.second;
		}
	return Dists;
}

void saveImage(const core::CHeightMap& vMap, const std::string& vPath, int vCoef = 1, std::pair<float, float> vClamp = std::make_pair(0, 0))
{
	Eigen::Matrix<float, -1, -1> Data;
	Data.resize(vMap.getWidth(), vMap.getHeight());
	for (int i = 0; i < vMap.getWidth(); i++)
		for (int k = 0; k < vMap.getHeight(); k++)
			Data.coeffRef(i, k) = vMap.getValueAt(i, k) * vCoef;

	if (vClamp.first != vClamp.second)
	{
		for (int i = 0; i < Data.rows(); i++)
			for (int k = 0; k < Data.cols(); k++)
				Data.coeffRef(i, k) -= vClamp.first;

		float Span = vClamp.second - vClamp.first;
		for (int i = 0; i < Data.rows(); i++)
			for (int k = 0; k < Data.cols(); k++)
				Data.coeffRef(i, k) = Data.coeffRef(i, k) / Span * 255;
	}

	common::saveImage(Data.cast<int>(), vPath);
}

TEST(TestImageInpaintingBasedOnNurbs, GenerateMap) 
{
	PC_t::Ptr pRaw = load(Path1);
	PC_t::Ptr pSub = load(Path2);

	int Degree = 3;
	int Refinement = 4;
	int Iteration = 10;
	core::CNurbsFitting Fitting;
	std::shared_ptr<pcl::on_nurbs::FittingSurface> Fit;
	EXPECT_TRUE(Fitting.run(pSub, Degree, Refinement, Iteration));
	Fitting.dumpFitting(Fit);
	hiveEventLogger::hiveOutputEvent("Fitting Finished");

	Eigen::Matrix<core::SPoint, -1, -1> Ctrlpts;
	EXPECT_TRUE(extractCtrlpts(Fit->m_nurbs, Ctrlpts));

	int SubNumber = 2;
	int SubLayer = 2;
	core::CMultilayerSurface* pSurface(new core::CMultilayerSurface(Degree));
	pSurface->setControlPoints(Ctrlpts);
	pSurface->setIsSaveMesh(true, "Mesh.obj");
	pSurface->setSubNumber(SubNumber);
	pSurface->setSubLayer(SubLayer);
	EXPECT_TRUE(pSurface->preCompute());

	std::vector<std::pair<float, Eigen::Vector2f>> Data;
	projPoints(Fit, pSurface, pRaw, Data);
	hiveEventLogger::hiveOutputEvent("Project Data Finished");

	int Width = 128;
	int Height = 128;
	core::CHeightMapGenerator MapGenerater;
	core::CHeightMap Map, Mask, Inpainted, InpaintedMask;
	EXPECT_TRUE(MapGenerater.generateBySurface(Data, Width, Height));
	MapGenerater.dumpHeightMap(Map);
	Map.generateMask(Mask);
	hiveEventLogger::hiveOutputEvent("Generate Map Finished");

	saveImage(Mask, "Mask.png", 255);
	saveImage(Map, "HeightMap.png", 5900);

	dataManagement::CImageInpainting Inpainter;
	Inpainter.run(Map, Inpainted);
	Inpainted.generateMask(InpaintedMask);
	hiveEventLogger::hiveOutputEvent("Inpainting Finished");

	saveImage(InpaintedMask, "InpaintedMask.png", 255);

	auto Extreme = calcExtreme(Map);
	hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Min: %1%, Max: %2%", Extreme.first, Extreme.second));
	Extreme = calcExtreme(Inpainted);
	hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Min: %1%, Max: %2%", Extreme.first, Extreme.second));

	for (int i = 29; i < 69; i++)
		for (int k = 16; k < 61; k++)
			Mask.setValueAt(0, i, k);
		
	for (int i = 115; i < 121; i++)
		for (int k = 33; k < 56; k++)
			Mask.setValueAt(0, i, k);

	for (int i = 120; i < 123; i++)
		for (int k = 34; k < 50; k++)
			Mask.setValueAt(0, i, k);

	for (int i = 0; i < Inpainted.getWidth(); i++)
		for (int k = 0; k < Inpainted.getHeight(); k++)
			if (Mask.getValueAt(i, k) == 1)
				Inpainted.setValueAt(0, i, k);

	saveImage(Inpainted, "InpaintedHeightMap.png", 5900);

	//saveImage(Inpainted, "InpaintedHeightMapx255.png", 1, Extreme);
	//saveImage(Map, "HeightMapx255.png", 1, Extreme);


}