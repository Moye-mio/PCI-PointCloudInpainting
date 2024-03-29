#include "pch.h"

const std::string Path1 = TESTMODEL_DIR + std::string("/Trimmed/Scene/WH_Scene_2.ply");

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

TEST(TestTiler, DT) 
{
	PC_t::Ptr pCloud(new PC_t);
	core::CTiler Tiler;
	EXPECT_FALSE(Tiler.run(pCloud, -1, -1, 10.0f));
	pCloud = loadPC(Path1);
	EXPECT_FALSE(Tiler.run(pCloud, -1, -1, 10.0f));
	EXPECT_FALSE(Tiler.run(pCloud, 2, -1, 10.0f));
	EXPECT_FALSE(Tiler.run(pCloud, 2, 3, 10.0f));
}

TEST(TestTiler, NT_4x4Scene)
{
	PC_t::Ptr pCloud = loadPC(Path1);
	std::vector<std::vector<int>> Tiles;
	std::vector<std::pair<int, int>> Coors;
	core::CTiler Tiler;
	EXPECT_TRUE(Tiler.run(pCloud, 4, 4, 0.1f));
	Tiler.dumpResult(Tiles, Coors);
	EXPECT_TRUE(Tiles.size() == 16 && Coors.size() == 16);

	for (int i = 0; i < Tiles.size(); i++)
	{
		const auto& t = Tiles[i];
		PC_t::Ptr pSave(new PC_t);
		for (auto e : t)
			pSave->emplace_back(pCloud->at(e));
		
		std::string Path = std::to_string(i) + ".ply";
		pcl::io::savePLYFileBinary(Path, *pSave);
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Save Cloud [%1%], Size [%2%]", Path, pSave->size()));
	}
}