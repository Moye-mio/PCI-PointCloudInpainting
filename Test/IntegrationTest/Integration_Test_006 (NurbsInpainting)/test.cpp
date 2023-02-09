#include "pch.h"

const std::string Path1 = TESTMODEL_DIR + std::string("/Trimmed/Walkway/WH_Walkway_Trim.ply");
const std::string Path2 = std::string("28.ply");

TEST(AllTest, PipeLineTest) 
{
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<dataManagement::IPCLoader>(hiveUtility::hiveGetFileSuffix(Path2));
	EXPECT_TRUE(pTileLoader);
	PC_t::Ptr pData = pTileLoader->loadDataFromFile(Path2);
	EXPECT_TRUE(pData->size());

	PC_t::Ptr pInpainted(new PC_t);
	dataManagement::CCloudInpaintingBasedOnNurbs Inpainter;
	EXPECT_TRUE(Inpainter.run(pData));
	Inpainter.dumpCloud(pInpainted);

	for (auto& e : *pInpainted)
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
		pInpainted->emplace_back(e);
	}

	pcl::io::savePLYFileBinary("NewCloud.ply", *pInpainted);
}