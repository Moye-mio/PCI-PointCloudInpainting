#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/Trimmed/CrossPlane/WH_CrossPlane.ply");
const std::string ModelPath2 = TESTMODEL_DIR + std::string("/Trimmed/CrossPlane/GT_CrossPlane.ply");
const std::string ModelPath3 = TESTMODEL_DIR + std::string("/Trimmed/Concave/WH_Concave.ply");
const std::string ModelPath4 = TESTMODEL_DIR + std::string("/Trimmed/Concave/GT_Concave.ply");
const std::string ModelPath5 = TESTMODEL_DIR + std::string("/Trimmed/Walkway/WH_Walkway.ply");
const std::string ModelPath6 = TESTMODEL_DIR + std::string("/Trimmed/Walkway/GT_Walkway.ply");

PC_t::Ptr generatePlanePC()
{
	PC_t::Ptr pCloud(new PC_t);
	for (float i = 0; i <= 100; i += 1.0f)
		for (float k = 0; k <= 100; k += 1.0f)
		{
			if (i >= 40 && i <= 60 && k >= 40 && k <= 60)
				continue;
			pCloud->push_back(Point_t(i, k, 1));
		}
	return pCloud;
}

void generatePlaneCP(Eigen::Matrix<core::SPoint, -1, -1>& vCPs, int vCoef = 1)
{
	vCPs.resize(11, 11);
	for (int i = 0; i < vCPs.rows(); i++)
		for (int k = 0; k < vCPs.cols(); k++)
		{
			//if (i <= 5)
			//	CPs.coeffRef(i, k) = core::SPoint(i * Coef, k * Coef, i * Coef);
			//if (i > 5)
			//	CPs.coeffRef(i, k) = core::SPoint(i * Coef, k * Coef, (10 - i) * Coef);
			vCPs.coeffRef(i, k) = core::SPoint(i * vCoef, k * vCoef, 0);
		}
}

void generateCrossPlaneCP(Eigen::Matrix<core::SPoint, -1, -1>& vCPs, int vCoef = 1)
{
	vCPs.resize(13, 13);
	int Start = -1;
	for (int i = Start; i < vCPs.rows() + Start; i++)
		for (int k = Start; k < vCPs.cols() + Start; k++)
		{
			if (i <= 5)
				vCPs.coeffRef(i - Start, k - Start) = core::SPoint(i * vCoef, k * vCoef, i * vCoef);
			if (i > 5)
				vCPs.coeffRef(i - Start, k - Start) = core::SPoint(i * vCoef, k * vCoef, (10 - i) * vCoef);
		}
}

void generateConcaveCP(Eigen::Matrix<core::SPoint, -1, -1>& voCPs, int vCoef = 1)
{
	voCPs.resize(18, 8);

	std::vector<core::SPoint> Points;
	for (int i = -1; i <= 6; i++)
	{
		for (int k = 6; k >= 0; k--)
			Points.emplace_back(core::SPoint(0.0f, i * vCoef, k * vCoef));

		for (int k = 1; k <= 5; k++)
			Points.emplace_back(core::SPoint(k * vCoef, i * vCoef, 0.0f));

		for (int k = 1; k <= 6; k++)
			Points.emplace_back(core::SPoint(5 * vCoef, i * vCoef, k * vCoef));
	}

	int Number = 0;
	for (int k = 0; k < voCPs.cols(); k++)
		for (int i = 0; i < voCPs.rows(); i++)
			voCPs.coeffRef(i, k) = Points[Number++];
}

core::SPoint transPoints(const Point_t& vPoint)
{
	core::SPoint Point(vPoint.x, vPoint.y, vPoint.z);
	return Point;
}

void saveMap2Image(const core::CHeightMap& vMap, const std::string& vPath, int vCoef = 1)
{
	Eigen::Matrix<float, -1, -1> Image;
	Image.resize(vMap.getWidth(), vMap.getHeight());
	for (int i = 0; i < vMap.getWidth(); i++)
		for (int k = 0; k < vMap.getHeight(); k++)
			Image.coeffRef(i, k) = vMap.getValueAt(i, k) * vCoef;

	common::saveImage(Image.cast<int>(), vPath);
}

PC_t::Ptr loadPC(const std::string& vPath)
{
	std::string FileName = hiveUtility::hiveLocateFile(vPath);
	_ASSERTE(!FileName.empty());

	PC_t::Ptr pCloud(new PC_t);
	int r = pcl::io::loadPLYFile<Point_t>(FileName, *pCloud);
	_ASSERTE(r != -1);
	_ASSERTE(pCloud->size());
	std::cout << "Model Point Size: " << pCloud->size() << std::endl;
	return pCloud;
}

void resizePC(PC_t::Ptr& vCloud, int vCoef = 1)
{
	for (auto& e : *vCloud)
	{
		e.x *= vCoef;
		e.y *= vCoef;
		e.z *= vCoef;
	}
}

void tuneMask(core::CHeightMap& vioMask, int vRes)
{
	int Rows = vioMask.getWidth();
	int Cols = vioMask.getHeight();

	int Tune = 14;
	/*if (vRes == 32)			Tune = 5;
	else if (vRes == 125)	Tune = 25;
	else if (vRes == 36)	Tune = 4;
	else if (vRes == 144)	Tune = 18;*/

	for (int i = 0; i < Rows; i++)
		for (int k = 0; k < Cols; k++)
			if (i < Tune / 2 || i > Rows - Tune / 2 - 1 || k < Tune || k > Cols - Tune - 1)
				vioMask.setValueAt(0, i, k);

	const core::CHeightMap Map = vioMask;

	for (int i = 1; i < Rows - 1; i++)
		for (int k = 1; k < Cols - 1; k++)
		{
			if (Map.getValueAt(i - 1, k) == 1 || Map.getValueAt(i, k - 1) == 1 || Map.getValueAt(i + 1, k) == 1 || Map.getValueAt(i, k + 1) == 1)
			vioMask.setValueAt(1, i, k);
		}
				

	vioMask.setValueAt(0, 16, 49);
}

void extractProjData(core::CMultilayerSurface* vSurface, const PC_t::Ptr& vCloud, std::vector<std::pair<float, Eigen::Vector2f>>& voData)
{
	voData.clear();
	std::cout << "Proj Data: " << std::endl;

	for (int i = 0; i < vCloud->size(); i++)
	{
		Point_t Point = vCloud->at(i);
		auto r = vSurface->calcProj(transPoints(Point));
		voData.emplace_back(std::make_pair(r->_Dist, r->_UV));
	}
}

//TEST(TESTSuface2PCMapper, Plane) 
//{
//	PC_t::Ptr pCloud = generatePlanePC();
//	Eigen::Matrix<core::SPoint, -1, -1> CPs;
//	generatePlaneCP(CPs, 10);
//
//	core::CMultilayerSurface* pSurface(new core::CMultilayerSurface(3));
//	pSurface->setControlPoints(CPs);
//	pSurface->setSubLayer(3);
//	pSurface->setIsSaveMesh(true);
//	pSurface->preCompute();
//
//	std::vector<std::pair<float, Eigen::Vector2f>> Data;
//	for (int i = 0; i < pCloud->size(); i++)
//	{
//		auto r = pSurface->calcProj(transPoints(pCloud->at(i)));
//
//		if (pCloud->at(i).x == pCloud->at(i).y)
//		{
//			std::cout << "Index: " << i << std::endl;
//			std::cout << "Dist: " << r->_Dist << ", UV: " << r->_UV << ", Proj: " << r->_Point << std::endl;
//		}
//
//		Data.emplace_back(std::make_pair(r->_Dist, r->_UV));
//	}
//
//	core::CHeightMap Map, Mask, Inpainted;
//	core::CHeightMapGenerator Generator;
//	Generator.generateBySurface(Data, 32, 32);
//	Generator.dumpHeightMap(Map);
//
//	/* Save HeightMap and Mask */
//	{
//		saveMap2Image(Map, "HeightMap.png");
//		Map.generateMask(Mask);
//		saveMap2Image(Mask, "Mask.png", 255);
//	}
//
//	/* generate Inpainted */
//	{
//		Eigen::Matrix<float, -1, -1> InpaintedImage;
//		InpaintedImage.resize(Map.getWidth(), Map.getHeight());
//		for (int i = 0; i < Map.getWidth(); i++)
//			for (int k = 0; k < Map.getHeight(); k++)
//				InpaintedImage.coeffRef(i, k) = 1;
//		Inpainted.setHeightMap(InpaintedImage);
//	}
//
//	std::shared_ptr<core::CMultilayerSurface> pTrSurface(pSurface);
//	PC_t::Ptr pNewCloud(new PC_t);
//	dataManagement::CSurface2CloudMapper Mapper;
//	EXPECT_TRUE(Mapper.setSurface(pTrSurface));
//	EXPECT_TRUE(Mapper.map2Cloud(Map, Inpainted, 100));
//	Mapper.dumpCloud(pNewCloud);
//
//	for (const auto& e : *pCloud)
//		pNewCloud->push_back(e);
//	pcl::io::savePLYFileBinary("NewCloud.ply", *pNewCloud);
//}

//TEST(TESTSuface2PCMapper, CrossPlane)
//{
//	int Res = 32;
//	PC_t::Ptr pCloud = loadPC(ModelPath);
//	resizePC(pCloud, 10);
//	Eigen::Matrix<core::SPoint, -1, -1> CPs;
//	generateCrossPlaneCP(CPs, 10);
//
//	core::CMultilayerSurface* pSurface(new core::CMultilayerSurface(3));
//	pSurface->setControlPoints(CPs);
//	pSurface->setSubLayer(3);
//	pSurface->setIsSaveMesh(true);
//	pSurface->preCompute();
//
//	std::vector<std::pair<float, Eigen::Vector2f>> Data;
//	extractProjData(pSurface, pCloud, Data);
//
//	core::CHeightMap Map, Mask, Inpainted, InpaintedMask;
//	core::CHeightMapGenerator Generator;
//	Generator.generateBySurface(Data, Res, Res);
//	Generator.dumpHeightMap(Map);
//
//	/* Save HeightMap and Mask */
//	{
//		saveMap2Image(Map, "HeightMap.png");
//		Map.generateMask(Mask);
//		tuneMask(Mask, Res);
//		saveMap2Image(Mask, "Mask.png", 255);
//
//		for (int i = 0; i < Map.getWidth(); i++)
//			for (int k = 0; k < Map.getHeight(); k++)
//				std::cout << "(" << i << ", " << k << "): " << Map.getValueAt(i, k) << std::endl;
//	}
//
//	/* generate Inpainted */
//	{
//		PC_t::Ptr pGTCloud = loadPC(ModelPath2);
//		resizePC(pGTCloud, 10);
//
//		std::vector<std::pair<float, Eigen::Vector2f>> Data2;
//		extractProjData(pSurface, pGTCloud, Data2);
//
//		core::CHeightMapGenerator Generator2;
//		Generator2.generateBySurface(Data2, Res, Res);
//		Generator2.dumpHeightMap(Inpainted);
//
//		saveMap2Image(Inpainted, "Inpainted.png");
//		Inpainted.generateMask(InpaintedMask);
//		saveMap2Image(InpaintedMask, "InpaintedMask.png", 255);
//
//		/* Output Inpainted Info */
//		hiveEventLogger::hiveOutputEvent("Inpainted Info: ");
//		for (int i = 0; i < Inpainted.getWidth(); i++)
//			for (int k = 0; k < Inpainted.getHeight(); k++)
//				hiveEventLogger::hiveOutputEvent(_FORMAT_STR3("Pixel [%1%, %2%]: %3%", i, k, Inpainted.getValueAt(i, k)));
//	}
//
//	std::shared_ptr<core::CMultilayerSurface> pTrSurface(pSurface);
//	PC_t::Ptr pNewCloud(new PC_t);
//	dataManagement::CSurface2CloudMapper Mapper;
//	EXPECT_TRUE(Mapper.setSurface(pTrSurface));
//	EXPECT_TRUE(Mapper.map2Cloud(Map, Mask, Inpainted, 100));
//	Mapper.dumpCloud(pNewCloud);
//
//	for (const auto& e : *pCloud)
//		pNewCloud->push_back(e);
//	pcl::io::savePLYFileBinary("NewCloud.ply", *pNewCloud);
//}

//TEST(TESTSuface2PCMapper, Concave)
//{
//	int ResX = 144;
//	int ResY = 64;
//	PC_t::Ptr pCloud = loadPC(ModelPath3);
//	Eigen::Matrix<core::SPoint, -1, -1> CPs;
//	generateConcaveCP(CPs, 20);
//
//	core::CMultilayerSurface* pSurface(new core::CMultilayerSurface(3));
//	pSurface->setControlPoints(CPs);
//	pSurface->setSubLayer(3);
//	pSurface->setIsSaveMesh(true);
//	pSurface->preCompute();
//
//	std::vector<std::pair<float, Eigen::Vector2f>> Data;
//	extractProjData(pSurface, pCloud, Data);
//
//	core::CHeightMap Map, Mask, Inpainted, InpaintedMask;
//	core::CHeightMapGenerator Generator;
//	Generator.generateBySurface(Data, ResX, ResY);
//	Generator.dumpHeightMap(Map);
//
//	/* Save HeightMap and Mask */
//	{
//		saveMap2Image(Map, "HeightMap.png");
//		Map.generateMask(Mask);
//		saveMap2Image(Mask, "MaskRaw.png", 255);
//		tuneMask(Mask, ResX);
//		saveMap2Image(Mask, "Mask.png", 255);
//
//		for (int i = 0; i < Map.getWidth(); i++)
//			for (int k = 0; k < Map.getHeight(); k++)
//				std::cout << "(" << i << ", " << k << "): " << Map.getValueAt(i, k) << std::endl;
//	}
//
//	/* generate Inpainted */
//	{
//		PC_t::Ptr pGTCloud = loadPC(ModelPath4);
//
//		std::vector<std::pair<float, Eigen::Vector2f>> Data2;
//		extractProjData(pSurface, pGTCloud, Data2);
//
//		core::CHeightMapGenerator Generator2;
//		Generator2.generateBySurface(Data2, ResX, ResY);
//		Generator2.dumpHeightMap(Inpainted);
//
//		saveMap2Image(Inpainted, "Inpainted.png");
//		Inpainted.generateMask(InpaintedMask);
//		saveMap2Image(InpaintedMask, "InpaintedMask.png", 255);
//
//		/* Output Inpainted Info */
//		hiveEventLogger::hiveOutputEvent("Inpainted Info: ");
//		for (int i = 0; i < Inpainted.getWidth(); i++)
//			for (int k = 0; k < Inpainted.getHeight(); k++)
//				hiveEventLogger::hiveOutputEvent(_FORMAT_STR3("Pixel [%1%, %2%]: %3%", i, k, Inpainted.getValueAt(i, k)));
//	}
//
//	std::shared_ptr<core::CMultilayerSurface> pTrSurface(pSurface);
//	PC_t::Ptr pNewCloud(new PC_t);
//	dataManagement::CSurface2CloudMapper Mapper;
//	EXPECT_TRUE(Mapper.setSurface(pTrSurface));
//	EXPECT_TRUE(Mapper.map2Cloud(Map, Mask, Inpainted, 25));
//	Mapper.dumpCloud(pNewCloud);
//
//	for (auto& e : *pNewCloud)
//	{
//		e.r = 253;
//		e.g = 199;
//		e.b = 83;
//	}
//
//	for (auto& e : *pCloud)
//	{
//		e.r = 255;
//		e.g = 255;
//		e.b = 255;
//		pNewCloud->push_back(e);
//	}
//	pcl::io::savePLYFileBinary("NewCloud.ply", *pNewCloud);
//}

TEST(TESTSuface2PCMapper, Walkway)
{
	Eigen::Matrix<Point_t, -1, -1> ControlPoints;
	std::shared_ptr<core::CMultilayerSurface> pSurface;
	std::vector<std::pair<float, Eigen::Vector2f>> DataWH, DataGT;
	dataManagement::CSurfaceGenerator SurfaceGenerator;
	EXPECT_TRUE(SurfaceGenerator.run(ModelPath6));
	SurfaceGenerator.dumpSurface(pSurface);
	SurfaceGenerator.dumpProjData(DataGT);
	SurfaceGenerator.dumpControlPoints(ControlPoints);

	/* load WH PC */
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<dataManagement::IPCLoader>(hiveUtility::hiveGetFileSuffix(ModelPath5));
	EXPECT_TRUE(pTileLoader);
	PC_t::Ptr pData = pTileLoader->loadDataFromFile(ModelPath5);
	EXPECT_TRUE(pData->size());

	extractProjData(pSurface.get(), pData, DataWH);

	int ResX = ControlPoints.rows() * 2;	// 53
	int ResY = ControlPoints.cols() * 2;	// 16

	core::CHeightMap Map, Mask, Inpainted, InpaintedMask;
	core::CHeightMapGenerator Generator;
	Generator.generateBySurface(DataWH, ResX, ResY);
	Generator.dumpHeightMap(Map);

	/* Save HeightMap and Mask */
	{
		saveMap2Image(Map, "HeightMap.png");
		Map.generateMask(Mask);
		saveMap2Image(Mask, "MaskRaw.png", 255);
		tuneMask(Mask, ResX);
		saveMap2Image(Mask, "Mask.png", 255);

		for (int i = 0; i < Map.getWidth(); i++)
			for (int k = 0; k < Map.getHeight(); k++)
				std::cout << "(" << i << ", " << k << "): " << Map.getValueAt(i, k) << std::endl;
	}

	/* generate Inpainted */
	{
		core::CHeightMapGenerator Generator2;
		Generator2.generateBySurface(DataGT, ResX, ResY);
		Generator2.dumpHeightMap(Inpainted);

		saveMap2Image(Inpainted, "Inpainted.png");
		Inpainted.generateMask(InpaintedMask);
		saveMap2Image(InpaintedMask, "InpaintedMask.png", 255);

		/* Output Inpainted Info */
		hiveEventLogger::hiveOutputEvent("Inpainted Info: ");
		for (int i = 0; i < Inpainted.getWidth(); i++)
			for (int k = 0; k < Inpainted.getHeight(); k++)
				hiveEventLogger::hiveOutputEvent(_FORMAT_STR3("Pixel [%1%, %2%]: %3%", i, k, Inpainted.getValueAt(i, k)));
	}

	PC_t::Ptr pNewCloud(new PC_t);
	dataManagement::CSurface2CloudMapper Mapper;
	EXPECT_TRUE(Mapper.setSurface(pSurface));
	EXPECT_TRUE(Mapper.map2Cloud(Map, Mask, Inpainted, 100));
	Mapper.dumpCloud(pNewCloud);

	for (auto& e : *pNewCloud)
	{
		e.r = 253;
		e.g = 199;
		e.b = 83;
	}

	for (auto& e : *pData)
	{
		e.r = 255;
		e.g = 255;
		e.b = 255;
		pNewCloud->push_back(e);
	}

	pcl::io::savePLYFileBinary("NewCloud.ply", *pNewCloud);
}


