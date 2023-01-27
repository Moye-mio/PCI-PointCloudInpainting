#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/Trimmed/CrossPlane/WH_CrossPlane.ply");
const std::string ModelPath2 = TESTMODEL_DIR + std::string("/Trimmed/CrossPlane/GT_CrossPlane.ply");

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

void tuneMask(core::CHeightMap& vioMask)
{
	int Rows = vioMask.getWidth();
	int Cols = vioMask.getHeight();

	int Tune = 5;

	for (int i = 0; i < Rows; i++)
		for (int k = 0; k < Cols; k++)
			if (i < Tune || i > Rows - Tune - 1 || k < Tune || k > Cols - Tune - 1)
				vioMask.setValueAt(0, i, k);
}

void extractProjData(core::CMultilayerSurface* vSurface, const PC_t::Ptr& vCloud, std::vector<std::pair<float, Eigen::Vector2f>>& voData)
{
	voData.clear();
	std::cout << "Proj Data: " << std::endl;

	for (int i = 0; i < vCloud->size(); i++)
	{
		Point_t Point = vCloud->at(i);
		auto r = vSurface->calcProj(transPoints(Point));
		if (r->_Dist > 50)
			std::cout << "Point Number " << i << ", (" << Point.x << ", " << Point.y << ", " << Point.z << "), D: " << r->_Dist << ", UV: " << r->_UV[0] << ", " << r->_UV[1] << ", Proj: " << r->_Point.x() << ", " << r->_Point.y() << ", " << r->_Point.z() << std::endl;
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

TEST(TESTSuface2PCMapper, CrossPlane)
{
	PC_t::Ptr pCloud = loadPC(ModelPath);
	resizePC(pCloud, 10);
	Eigen::Matrix<core::SPoint, -1, -1> CPs;
	generateCrossPlaneCP(CPs, 10);

	core::CMultilayerSurface* pSurface(new core::CMultilayerSurface(3));
	pSurface->setControlPoints(CPs);
	pSurface->setSubLayer(3);
	pSurface->setIsSaveMesh(true);
	pSurface->preCompute();

	std::vector<std::pair<float, Eigen::Vector2f>> Data;
	extractProjData(pSurface, pCloud, Data);

	core::CHeightMap Map, Mask, Inpainted, InpaintedMask;
	core::CHeightMapGenerator Generator;
	Generator.generateBySurface(Data, 32, 32);
	Generator.dumpHeightMap(Map);

	/* Save HeightMap and Mask */
	{
		saveMap2Image(Map, "HeightMap.png");
		Map.generateMask(Mask);
		tuneMask(Mask);
		saveMap2Image(Mask, "Mask.png", 255);

		for (int i = 0; i < Map.getWidth(); i++)
			for (int k = 0; k < Map.getHeight(); k++)
				std::cout << "(" << i << ", " << k << "): " << Map.getValueAt(i, k) << std::endl;
	}

	/* generate Inpainted */
	{
		PC_t::Ptr pGTCloud = loadPC(ModelPath2);
		resizePC(pGTCloud, 10);

		std::vector<std::pair<float, Eigen::Vector2f>> Data2;
		extractProjData(pSurface, pGTCloud, Data2);

		core::CHeightMapGenerator Generator2;
		Generator2.generateBySurface(Data2, 32, 32);
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

	std::shared_ptr<core::CMultilayerSurface> pTrSurface(pSurface);
	PC_t::Ptr pNewCloud(new PC_t);
	dataManagement::CSurface2CloudMapper Mapper;
	EXPECT_TRUE(Mapper.setSurface(pTrSurface));
	EXPECT_TRUE(Mapper.map2Cloud(Map, Mask, Inpainted, 100));
	Mapper.dumpCloud(pNewCloud);

	for (const auto& e : *pCloud)
		pNewCloud->push_back(e);
	pcl::io::savePLYFileBinary("NewCloud.ply", *pNewCloud);
}
