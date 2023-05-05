#include "pch.h"

const std::string Path1 = TESTMODEL_DIR + std::string("/Trimmed/Scene/WH_Scene_1.ply");
const std::string Path2 = TESTMODEL_DIR + std::string("/Trimmed/Scene/WH_Scene_2.ply");
const std::string Path3 = TESTMODEL_DIR + std::string("/Trimmed/Scene/RAW_Scene_Seg_1.ply");
const std::string Path4 = TESTMODEL_DIR + std::string("/Trimmed/Scene/RAW_Scene_Seg_1_Sub.ply");
const std::string Path5 = TESTMODEL_DIR + std::string("/Trimmed/Scene/RAW_Scene_GT.ply");
const std::string Path6 = TESTMODEL_DIR + std::string("/Trimmed/Scene/RAW_Scene_Seg_2.ply");
const std::string Path7 = TESTMODEL_DIR + std::string("/Trimmed/Scene/RAW_Scene_Seg_2_Sub.ply");
const std::string Path8 = TESTMODEL_DIR + std::string("/Trimmed/Terrain/ASCII/Sample_4_120_Sub_5.ply");
const std::string Path9 = TESTMODEL_DIR + std::string("/Trimmed/Terrain/ASCII/Sample_5_120_Seg_Sub_6_ASCII.ply");
const std::string Path10 = TESTMODEL_DIR + std::string("/Trimmed/Terrain/ASCII/Sample_5_120_Seg_ASCII.ply");
const std::string Path11 = TESTMODEL_DIR + std::string("/Trimmed/Terrain/ASCII/Sample_5_120_ASCII.ply");
const std::string Path12 = TESTMODEL_DIR + std::string("/ExperimentResult/Tiles/TargetGT.ply");
const std::string Path13 = TESTMODEL_DIR + std::string("/ExperimentResult/Tiles/TargetRaw.ply");
const std::string Path14 = TESTMODEL_DIR + std::string("/ExperimentResult/Tiles/TargetSub.ply");
const std::string Path15 = TESTMODEL_DIR + std::string("/Experiment/MultiTile_GT.ply");
const std::string Path16 = TESTMODEL_DIR + std::string("/Experiment/MultiTile_Sub.ply");
const std::string Path17 = TESTMODEL_DIR + std::string("/Experiment/MultiTile_WH.ply");
const std::string Path18 = TESTMODEL_DIR + std::string("/Experiment/WalkWay/WH_Walkway_Trim_0.ply");
const std::string Path19 = TESTMODEL_DIR + std::string("/Experiment/WalkWay/WH_Walkway_Trim_0_Sub_6.ply");
const std::string Path20 = TESTMODEL_DIR + std::string("/Experiment/WalkWay/GT_Walkway_Trim.ply");
const std::string Path21 = TESTMODEL_DIR + std::string("/Experiment/WalkWayRegion/WalkWayRegion2_Raw.ply");
const std::string Path22 = TESTMODEL_DIR + std::string("/Experiment/WalkWayRegion/WalkWayRegion2_WH.ply");
const std::string Path23 = TESTMODEL_DIR + std::string("/Experiment/WalkWayRegion/WalkWayRegion2_WH_Sub_6.ply");
const std::string Path24 = TESTMODEL_DIR + std::string("/Experiment/WalkWayRegion/WalkWayRegion2_Raw_Sub_6.ply");
const std::string Path25 = TESTMODEL_DIR + std::string("/Experiment/WalkWayRegion/WalkWayRegion2_WH_Local.ply");
const std::string Path26 = TESTMODEL_DIR + std::string("/Experiment/WalkWayRegion/WalkWayRegion2_GT_Local.ply");
const std::string Path27 = TESTMODEL_DIR + std::string("/Experiment/WalkWayRegion/WalkWayRegion2_GT_Local_Sub_6.ply");
const std::string Path28 = TESTMODEL_DIR + std::string("/Experiment/RockRegion/Rock_2_GT_Local.ply");
const std::string Path29 = TESTMODEL_DIR + std::string("/Experiment/RockRegion/Rock_2_WH_Local.ply");
const std::string Path30 = TESTMODEL_DIR + std::string("/Experiment/RockRegion/Rock_2_GT_Local_Sub_6.ply");
const std::string Path31 = TESTMODEL_DIR + std::string("/Experiment/TerrainRegion/WinterTerrain_GT_Local.ply");
const std::string Path32 = TESTMODEL_DIR + std::string("/Experiment/TerrainRegion/WinterTerrain_WH_Local.ply");
const std::string Path33 = TESTMODEL_DIR + std::string("/Experiment/TerrainRegion/WinterTerrain_GT_Local_Sub_6.ply");
const std::string Path34 = TESTMODEL_DIR + std::string("/Experiment/Addition/Rock_0_Local.ply");
const std::string Path35 = TESTMODEL_DIR + std::string("/Experiment/Addition/Rock_0_Local_Sub_6.ply");
const std::string Path36 = TESTMODEL_DIR + std::string("/Experiment/Addition/Rock_0_WH.ply");
const std::string Path37 = TESTMODEL_DIR + std::string("/Experiment/TerrainRegion/WinterTerrain2-1_Local.ply");
const std::string Path38 = TESTMODEL_DIR + std::string("/Experiment/TerrainRegion/WinterTerrain2-1_Local_Sub_6.ply");
const std::string Path39 = TESTMODEL_DIR + std::string("/Experiment/TerrainRegion/WinterTerrain2-1_WH_Local.ply");

PC_t::Ptr load(const std::string& vPath)
{
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<dataManagement::IPCLoader>(hiveUtility::hiveGetFileSuffix(vPath));
	_HIVE_EARLY_RETURN(pTileLoader == nullptr, "TileLoader is nullptr", nullptr);
	PC_t::Ptr pCloud = pTileLoader->loadDataFromFile(vPath);
	_HIVE_EARLY_RETURN(pCloud->size() == 0, "Cloud is empty", nullptr);
	hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("[%1%] load successfully, Size [%2%]", vPath, pCloud->size()));
	return pCloud;
}

void normalize(PC_t::Ptr& vioCloud)
{
	float MinX = FLT_MAX;
	float MinY = FLT_MAX;
	for (const auto& e : *vioCloud)
	{
		MinX = (MinX < e.x) ? MinX : e.x;
		MinY = (MinY < e.y) ? MinY : e.y;
	}

	for (auto& e : *vioCloud)
	{
		e.x -= MinX;
		e.y -= MinY;
	}
}

//TEST(TestTilesInpainting, NT)
//{
//	PC_t::Ptr pRaw = load(Path3);
//	PC_t::Ptr pSub = load(Path4);
//
//	int SizeX = 4;
//	int SizeY = 4;
//	float Rate = 0.1f;
//
//	dataManagement::CTilesInpainting Inpainter;
//	EXPECT_TRUE(Inpainter.run(pRaw, pSub, SizeX, SizeY, Rate));
//}

//TEST()
//{
//	PC_t::Ptr pOne = load(Path17);
//	normalize(pOne);
//	pcl::io::savePLYFileBinary("Result/comparison/WH.ply", *pOne);
//}

//TEST(TestTilesInpainting, NT_BasedOnGT)
//{
//	PC_t::Ptr pRaw = load(Path17);
//	PC_t::Ptr pSub = load(Path16);
//	PC_t::Ptr pGT = load(Path15);
//	PC_t::Ptr pResult(new PC_t);
//
//	normalize(pRaw);
//	normalize(pSub);
//	normalize(pGT);
//
//	int SizeX = 1;
//	int SizeY = 2;
//	float Rate = 0.2f;
//
//	dataManagement::CTilesInpaintingBasedOnGT Inpainter;
//	Inpainter.setTilesInfo(Rate, SizeX, SizeY);
//	Inpainter.setNurbsInfo(3, 4, 10, 2, 2);
//	Inpainter.setMapInfo(64, 64, 85);
//	EXPECT_TRUE(Inpainter.run(pRaw, pSub, pGT));
//	Inpainter.dumpCloud(pResult);
//
//	pcl::io::savePLYFileBinary("Result/Result.ply", *pResult);
//	for (const auto& e : *pRaw)
//		pResult->emplace_back(Point_t(e.x, e.y, e.z, (std::uint8_t)255, (std::uint8_t)255, (std::uint8_t)255, (std::uint8_t)255));
//	pcl::io::savePLYFileBinary("Result/Merge.ply", *pResult);
//
//	for (auto& e : *pResult)
//	{
//		e.r = (std::uint8_t)255;
//		e.g = (std::uint8_t)255;
//		e.b = (std::uint8_t)255;
//	}
//	pcl::io::savePLYFileBinary("Result/MergeNoColor.ply", *pResult);
//}

//TEST(TestTilesInpainting, Extract)
//{
//	PC_t::Ptr pCloud = load("D:\\Projects\\PCI\\Models\\ExperimentResult\\WalkWayRegion\\WalkWayRegion_0.ply");
//	PC_t::Ptr pCloud2 = load(Path25);
//	PC_t::Ptr pNew(new PC_t);
//	for (auto& e : *pCloud)
//		for (auto& ee : *pCloud2)
//			if (!(e.x == ee.x && e.y == ee.y && e.z == ee.z))
//				pNew->emplace_back(e);
//	pcl::io::savePLYFileBinary("D:\\Projects\\PCI\\Models\\ExperimentResult\\WalkWayRegion\\WalkWayRegion_1.ply", *pNew);
//}

TEST(TestTilesInpainting, NT_OneTile)
{
	PC_t::Ptr pRaw = load(Path39);
	PC_t::Ptr pSub = load(Path38);
	PC_t::Ptr pGT = load(Path37);
	PC_t::Ptr pResult(new PC_t);

	int SizeX = 1;
	int SizeY = 1;
	float Rate = 0.0f;

	dataManagement::CTilesInpaintingBasedOnGT Inpainter;
	Inpainter.setTilesInfo(Rate, SizeX, SizeY);
	Inpainter.setNurbsInfo(3, 4, 10, 2, 2);
	Inpainter.setMapInfo(64, 64, 10);
	EXPECT_TRUE(Inpainter.run(pRaw, pSub, pGT));
	Inpainter.dumpCloud(pResult);

	pcl::io::savePLYFileBinary("Result/Addition/WinterTerrain2Result.ply", *pResult);
	for (const auto& e : *pRaw)
		pResult->emplace_back(Point_t(e.x, e.y, e.z, (std::uint8_t)255, (std::uint8_t)255, (std::uint8_t)255, (std::uint8_t)255));
	pcl::io::savePLYFileBinary("Result/Addition/WinterTerrain2Merge.ply", *pResult);

	for (auto& e : *pResult)
	{
		e.r = (std::uint8_t)0;
		e.g = (std::uint8_t)0;
		e.b = (std::uint8_t)0;
	}
	pcl::io::savePLYFileBinary("Result/Addition/WinterTerrain2MergeNoColor.ply", *pResult);
}