#include "pch.h"

const std::string Path1 = TESTMODEL_DIR + std::string("/Trimmed/Scene/WH_Scene_1.ply");
const std::string Path2 = TESTMODEL_DIR + std::string("/Trimmed/Scene/WH_Scene_2.ply");

PC_t::Ptr load(const std::string& vPath)
{
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<dataManagement::IPCLoader>(hiveUtility::hiveGetFileSuffix(vPath));
	_HIVE_EARLY_RETURN(pTileLoader == nullptr, "TileLoader is nullptr", nullptr);
	PC_t::Ptr pCloud = pTileLoader->loadDataFromFile(vPath);
	_HIVE_EARLY_RETURN(pCloud->size() == 0, "Cloud is empty", nullptr);
	return pCloud;
}

TEST(TestTilesInpainting, NT)
{
	PC_t::Ptr pRaw = load(Path1);
	PC_t::Ptr pSub = load(Path2);

	int SizeX = 4;
	int SizeY = 4;
	float Rate = 0.1f;

	dataManagement::CTilesInpainting Inpainter;
	EXPECT_TRUE(Inpainter.run(pRaw, pSub, SizeX, SizeY, Rate));
}