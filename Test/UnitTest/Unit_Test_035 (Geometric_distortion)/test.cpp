#include "pch.h"

const std::string Path1 = TESTMODEL_DIR + std::string("/Trimmed/CrossPlane/GT_CrossPlane.ply");
const std::string Path2 = TESTMODEL_DIR + std::string("/Trimmed/Scene/RAW_Scene_GT.ply");
const std::string Path3 = TESTMODEL_DIR + std::string("/Trimmed/Scene/Result/MergeBasedOnGT.ply");
const std::string Path4 = TESTMODEL_DIR + std::string("/Trimmed/Scene/Result/CurveShorteningFlow.ply");
const std::string Path5 = TESTMODEL_DIR + std::string("/Trimmed/Scene/Result/MergeBasedOnGT-Culled.ply");
const std::string Path6 = TESTMODEL_DIR + std::string("/Trimmed/Scene/Result/Meshlab.ply");
const std::string Path7 = TESTMODEL_DIR + std::string("/Trimmed/Scene/Result/MeshFix.ply");
const std::string Path8 = TESTMODEL_DIR + std::string("/ExperimentResult/GT/Sub/ASCII/Terrain_1_GT_Sub_8_ASCII.ply");
const std::string Path9 = TESTMODEL_DIR + std::string("/ExperimentResult/CCF/Sub/Merge/Terrain_5_CCF_Sub_8_Merge.ply");
const std::string Path9_1 = TESTMODEL_DIR + std::string("/ExperimentResult/CCF/Sub/Merge/Terrain_1_CCF_Sub_8_Merge.ply");
const std::string Path10 = TESTMODEL_DIR + std::string("/ExperimentResult/MultiTilesInpainting/OneTile_Sub_8.ply");
const std::string Path11 = TESTMODEL_DIR + std::string("/ExperimentResult/MultiTilesInpainting/MultiTile_Sub_8.ply");
const std::string Path12 = TESTMODEL_DIR + std::string("/ExperimentResult/DepthMap/Merge_5_DM.ply");

using namespace pcl::geometric_quality;

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

void pointscale(const PC_t::Ptr& vCloud)
{
	for (auto& e : *vCloud)
	{
		e.x *= 100;
		e.y *= 100;
		e.z *= 100;
	}
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

TEST(TestPNSR, 1)
{
	PC_t::Ptr pCloud1 = loadPC(Path12);
	PC_t::Ptr pCloud2 = loadPC(Path8);

	normalize(pCloud1);
	normalize(pCloud2);

	core::CSimilarityEstimator Estimator;
	auto r1 = Estimator.compute(pCloud1, pCloud2, core::ESimilarityMode::GPSNR);
	auto r2 = Estimator.compute(pCloud1, pCloud2, core::ESimilarityMode::NSHD);
	EXPECT_TRUE(r1.has_value());
	EXPECT_TRUE(r2.has_value());
	std::cout << "GPSNR: " << r1.value() << std::endl;
	std::cout << "NSHD: " << r2.value() << std::endl;
}