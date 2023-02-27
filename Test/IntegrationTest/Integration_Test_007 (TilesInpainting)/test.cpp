#include "pch.h"

const std::string Path1 = TESTMODEL_DIR + std::string("/Trimmed/Scene/WH_Scene_1.ply");
const std::string Path2 = TESTMODEL_DIR + std::string("/Trimmed/Scene/WH_Scene_2.ply");
const std::string Path3 = TESTMODEL_DIR + std::string("/Trimmed/Scene/RAW_Scene_Seg_1.ply");
const std::string Path4 = TESTMODEL_DIR + std::string("/Trimmed/Scene/RAW_Scene_Seg_1_Sub.ply");
const std::string Path5 = TESTMODEL_DIR + std::string("/Trimmed/Scene/RAW_Scene_GT.ply");
const std::string Path6 = TESTMODEL_DIR + std::string("/Trimmed/Scene/RAW_Scene_Seg_2.ply");
const std::string Path7 = TESTMODEL_DIR + std::string("/Trimmed/Scene/RAW_Scene_Seg_2_Sub.ply");

PC_t::Ptr load(const std::string& vPath)
{
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<dataManagement::IPCLoader>(hiveUtility::hiveGetFileSuffix(vPath));
	_HIVE_EARLY_RETURN(pTileLoader == nullptr, "TileLoader is nullptr", nullptr);
	PC_t::Ptr pCloud = pTileLoader->loadDataFromFile(vPath);
	_HIVE_EARLY_RETURN(pCloud->size() == 0, "Cloud is empty", nullptr);
	hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("[%1%] load successfully, Size [%2%]", vPath, pCloud->size()));
	return pCloud;
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

TEST(TestTilesInpainting, NT_BasedOnGT)
{
	PC_t::Ptr pRaw = load(Path6);
	PC_t::Ptr pSub = load(Path7);
	PC_t::Ptr pGT = load(Path5);
	PC_t::Ptr pResult(new PC_t);

	int SizeX = 4;
	int SizeY = 4;
	float Rate = 0.2f;

	dataManagement::CTilesInpaintingBasedOnGT Inpainter;
	Inpainter.setTilesInfo(Rate, SizeX, SizeY);
	Inpainter.setNurbsInfo(3, 4, 10, 2, 2);
	Inpainter.setMapInfo(64, 64, 100);
	EXPECT_TRUE(Inpainter.run(pRaw, pSub, pGT));
	Inpainter.dumpCloud(pResult);

	pcl::io::savePLYFileBinary("Result.ply", *pResult);
	for (const auto& e : *pRaw)
		pResult->emplace_back(Point_t(e.x, e.y, e.z, (std::uint8_t)255, (std::uint8_t)255, (std::uint8_t)255, (std::uint8_t)255));
	pcl::io::savePLYFileBinary("Merge.ply", *pResult);

	for (auto& e : *pResult)
	{
		e.r = (std::uint8_t)255;
		e.g = (std::uint8_t)255;
		e.b = (std::uint8_t)255;
	}
	pcl::io::savePLYFileBinary("MergeNoColor.ply", *pResult);
}