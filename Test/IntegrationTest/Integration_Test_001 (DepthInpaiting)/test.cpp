#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/Pyramid.ply");
const std::string ModelPath2 = TESTMODEL_DIR + std::string("/SlantPyramid.ply");
const std::string ModelPath3 = TESTMODEL_DIR + std::string("/KA_233_Simplified.las");
const std::string ModelPath4 = TESTMODEL_DIR + std::string("/HillRoad.las");
const std::string ModelPath5 = TESTMODEL_DIR + std::string("/Road_Seg_Sim.ply");
const std::string ModelPath6 = TESTMODEL_DIR + std::string("/Hill2.ply");
const std::string ModelPath7 = TESTMODEL_DIR + std::string("/Trimmed/CrossPlane/WH_CrossPlane.ply");
const std::string ModelPath8 = TESTMODEL_DIR + std::string("/SemiSphere.ply");
const std::string ModelPath9 = TESTMODEL_DIR + std::string("/ExperimentResult/Hole/Sub/Uniform/Terrain_5_Hole_Sub_8_Uniform.ply");
const std::string ModelPath10 = TESTMODEL_DIR + std::string("/Experiment/WalkWay/WH_Walkway_Trim_0.ply");
const std::string ModelPath11 = TESTMODEL_DIR + std::string("/Experiment/WalkWayRegion/WalkWayRegion2_WH.ply");
const std::string ModelPath12 = TESTMODEL_DIR + std::string("/Experiment/WalkWayRegion/WalkWayRegion2_Raw.ply");
const std::string ModelPath13 = TESTMODEL_DIR + std::string("/Experiment/RockRegion/Rock_2_WH_Local.ply");
const std::string ModelPath14 = TESTMODEL_DIR + std::string("/Experiment/RockRegion/Rock_2_GT_Local.ply");
const std::string ModelPath15 = TESTMODEL_DIR + std::string("/Experiment/TerrainRegion/WinterTerrain_WH_Local.ply");
const std::string ModelPath16 = TESTMODEL_DIR + std::string("/Experiment/TerrainRegion/WinterTerrain_GT_Local.ply");
const std::string ModelPath17 = TESTMODEL_DIR + std::string("/Experiment/Addition/Rock_0_WH_1.ply");
const std::string ModelPath18 = TESTMODEL_DIR + std::string("/Experiment/Addition/Rock_0.ply");
const std::string ModelPath19 = TESTMODEL_DIR + std::string("/Experiment/TerrainRegion/WinterTerrain2-1.ply");


class TestDepthInpainting : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}

	float m_Epsilon = 0.00001f;
};

TEST_F(TestDepthInpainting, NT_DepthInpainting)
{
	const auto Path = ModelPath17;

	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<dataManagement::IPCLoader>(hiveUtility::hiveGetFileSuffix(Path));
	ASSERT_TRUE(pTileLoader);
	PC_t::Ptr pData = pTileLoader->loadDataFromFile(Path);
	ASSERT_TRUE(pData);

	core::CAABBEstimation Estimation(pData);
	core::SAABB Box = Estimation.compute();

	PC_t::Ptr pOutput;
	dataManagement::CDepthInpaiting DepthInpainting;
	DepthInpainting.run(pData, pOutput);

	EXPECT_TRUE(pOutput->size());
	pcl::io::savePLYFileBinary("Result/Output.ply", *pOutput);
}

TEST_F(TestDepthInpainting, NT_InpaintingBasedOnGT)
{
	{
		std::string Path = ModelPath19;
		auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<dataManagement::IPCLoader>(hiveUtility::hiveGetFileSuffix(Path));
		ASSERT_TRUE(pTileLoader);
		PC_t::Ptr pData = pTileLoader->loadDataFromFile(Path);
		ASSERT_TRUE(pData);

		/*for (auto& e : *pData)
		{
			float temp = e.z;
			e.z = e.y;
			e.y = temp;
		}*/

		/*float MinX = FLT_MAX;
		float MinY = FLT_MAX;
		float MinZ = FLT_MAX;
		for (const auto& e : *pData)
		{
			MinX = (e.x < MinX) ? e.x : MinX;
			MinY = (e.y < MinY) ? e.y : MinY;
			MinZ = (e.z < MinZ) ? e.z : MinZ;
		}

		for (auto& e : *pData)
		{
			e.x -= MinX;
			e.y -= MinY;
			e.z -= MinZ;
		}*/

		for (auto& e : *pData)
		{
			e.x /= 2;
			e.y /= 2;
			//e.z /= 100;
		}

		pcl::io::savePLYFileBinary(ModelPath19, *pData);
	}

	const auto Model = ModelPath17;
	const auto GTModel = ModelPath18;
	
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<dataManagement::IPCLoader>(hiveUtility::hiveGetFileSuffix(Model));
	ASSERT_TRUE(pTileLoader);
	PC_t::Ptr pData = pTileLoader->loadDataFromFile(Model);
	ASSERT_TRUE(pData);

	auto* pTileLoader2 = hiveDesignPattern::hiveGetOrCreateProduct<dataManagement::IPCLoader>(hiveUtility::hiveGetFileSuffix(GTModel));
	ASSERT_TRUE(pTileLoader2);
	PC_t::Ptr pData2 = pTileLoader2->loadDataFromFile(GTModel);
	ASSERT_TRUE(pData2);

	/*for (int i = 0; i < pData->size(); i++)
	{
		float t = pData->at(i).y;
		pData->at(i).y = pData->at(i).z;
		pData->at(i).z = t;
	}*/

	//core::CAABBEstimation Estimation(pData);
	//core::SAABB Box = Estimation.compute();

	//for (int i = 0; i < pData->size(); i++)
	//{
	//	pData->at(i).x -= Box._Min[0];
	//	pData->at(i).y -= Box._Min[1];
	//	pData->at(i).z -= Box._Min[2];
	//	/*pData->at(i).r = (std::uint8_t)255;
	//	pData->at(i).g = (std::uint8_t)255;
	//	pData->at(i).b = (std::uint8_t)255;*/

	//	/*float t = pData->at(i).y;
	//	pData->at(i).y = pData->at(i).z;
	//	pData->at(i).z = t;*/
	//}

	PC_t::Ptr pOutput;
	dataManagement::CDepthInpaiting DepthInpainting;
	DepthInpainting.run(pData, pOutput);

	/* Create HeightMap */
	core::CHeightMap HeightMap;
	core::CHeightMapGenerator HGenerator;
	HGenerator.setCloud(pData);
	HGenerator.generate(64, 64);			/* Magic Number */
	HGenerator.dumpHeightMap(HeightMap);
	_ASSERTE(HeightMap.isValid());

	/* Create HeightMap */
	core::CHeightMap HeightMap2, HeightMapInpainted2;
	core::CHeightMapGenerator HGenerator2;
	HGenerator2.setCloud(pData2);
	HGenerator2.generate(64, 64);			/* Magic Number */
	HGenerator2.dumpHeightMap(HeightMap2);
	_ASSERTE(HeightMap2.isValid());

	{
		cv::Mat Save(cv::Size(HeightMap.getWidth(), HeightMap.getHeight()), CV_8UC1);
		for (int i = 0; i < Save.rows; i++)
			for (int k = 0; k < Save.cols; k++)
				Save.at<unsigned char>(i, k) = (unsigned char)(HeightMap.getValueAt(i, k) * 1);
		cv::imwrite("Save.png", Save);
		cv::Mat Save2(cv::Size(HeightMap2.getWidth(), HeightMap2.getHeight()), CV_8UC1);
		for (int i = 0; i < Save2.rows; i++)
			for (int k = 0; k < Save2.cols; k++)
				Save2.at<unsigned char>(i, k) = (unsigned char)(HeightMap2.getValueAt(i, k) * 1);
		cv::imwrite("Save2.png", Save2);
	}

	{
		std::vector<Eigen::Vector2i> v;
		for (int i = 0; i < HeightMap2.getWidth(); i++)
			for (int k = 0; k < HeightMap2.getHeight(); k++)
				if (HeightMap2.isEmptyValue(i, k))
				{
					HeightMap2.setValueAt(0, i, k);
					v.emplace_back(Eigen::Vector2i(i, k));
				}

		for (const auto& e : v)
		{
			HeightMap.setValueAt(0, e[0], e[1]);
		}
	}

	/* Map to Cloud */
	core::CAABBEstimation Estimation(pData2);
	core::SAABB Box = Estimation.compute();
	_ASSERTE(Box.isValid());
	core::CHeightMap2PCMapper Mapper;
	Mapper.map2PC(pOutput, std::make_pair(HeightMap, HeightMap2), Box, 25);	/* Magic Number */

	pcl::io::savePLYFileBinary<Point_t>("Result/Addition/Inpainted2.ply", *pOutput);
	*pOutput += *pData;
	pcl::io::savePLYFileBinary<Point_t>("Result/Addition/Merge2.ply", *pOutput);

}