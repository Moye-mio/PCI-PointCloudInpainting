#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/Walkway.ply");
const std::string ModelPath2 = TESTMODEL_DIR + std::string("/Trimmed/Walkway/WH_Walkway.ply");
const std::string ModelPath3 = TESTMODEL_DIR + std::string("/Trimmed/Walkway/GT_Walkway.ply");

void extractProjData(core::CMultilayerSurface* vSurface, const PC_t::Ptr& vCloud, std::vector<std::pair<float, Eigen::Vector2f>>& voData)
{
	voData.clear();
	std::cout << "Proj Data: " << std::endl;

	for (int i = 0; i < vCloud->size(); i++)
	{
		Point_t Point = vCloud->at(i);
		auto r = vSurface->calcProj(core::SPoint(Point.x, Point.y, Point.z));
		if (r.has_value() == false) continue;
		voData.emplace_back(std::make_pair(r->_Dist, r->_UV));
	}
}

TEST(TestSurfaceProjecting, NT) 
{
	Eigen::Matrix<Point_t, -1, -1> ControlPoints;
	std::shared_ptr<core::CMultilayerSurface> pSurface;
	std::vector<std::pair<float, Eigen::Vector2f>> DataWH, DataGT;
	dataManagement::CSurfaceGenerator SurfaceGenerator;
	EXPECT_TRUE(SurfaceGenerator.run(ModelPath));
	SurfaceGenerator.dumpSurface(pSurface);
	SurfaceGenerator.dumpProjData(DataGT);
	SurfaceGenerator.dumpControlPoints(ControlPoints);

	/* load WH PC */
	//auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<dataManagement::IPCLoader>(hiveUtility::hiveGetFileSuffix(ModelPath2));
	//EXPECT_TRUE(pTileLoader);
	//PC_t::Ptr pData = pTileLoader->loadDataFromFile(ModelPath2);
	//EXPECT_TRUE(pData->size());

	///* generate Surface */
	//extractProjData(pSurface.get(), pData, DataWH);
	//
	//int ResX = ControlPoints.rows();
	//int ResY = ControlPoints.cols();
	//core::CHeightMap Map, Mask, Inpainted, InpaintedMask;
	//core::CHeightMapGenerator MapGenerator;
	//MapGenerator.generateBySurface(DataWH, ResX, ResY);
	//MapGenerator.dumpHeightMap(Map);

	//core::CHeightMapGenerator Generator2;
	//Generator2.generateBySurface(DataGT, ResX, ResY);
	//Generator2.dumpHeightMap(Inpainted);

}