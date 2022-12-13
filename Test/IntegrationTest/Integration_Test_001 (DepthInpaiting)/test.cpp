#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/Pyramid.ply");
const std::string ModelPath2 = TESTMODEL_DIR + std::string("/SlantPyramid.ply");
const std::string ModelPath3 = TESTMODEL_DIR + std::string("/KA_233_Simplified.las");
const std::string ModelPath4 = TESTMODEL_DIR + std::string("/HillRoad.las");
const std::string ModelPath5 = TESTMODEL_DIR + std::string("/Road_Seg_Sim.ply");
const std::string ModelPath6 = TESTMODEL_DIR + std::string("/Hill2.ply");
const std::string ModelPath7 = TESTMODEL_DIR + std::string("/CrossPlane.ply");
const std::string ModelPath8 = TESTMODEL_DIR + std::string("/SemiSphere.ply");

class TestDepthInpainting : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}

	float m_Epsilon = 0.00001;
};

TEST_F(TestDepthInpainting, NT_)
{
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<dataManagement::IPCLoader>(hiveUtility::hiveGetFileSuffix(ModelPath5));
	ASSERT_TRUE(pTileLoader);
	PC_t::Ptr pData = pTileLoader->loadDataFromFile(ModelPath5);
	ASSERT_TRUE(pData);

	for (int i = 0; i < pData->size(); i++)
	{
		float t = pData->at(i).y;
		pData->at(i).y = pData->at(i).z;
		pData->at(i).z = t;
	}

	core::CAABBEstimation Estimation(pData);
	core::SAABB Box = Estimation.compute();

	for (int i = 0; i < pData->size(); i++)
	{
		pData->at(i).x -= Box._Min[0];
		pData->at(i).y -= Box._Min[1];
		pData->at(i).z -= Box._Min[2];
		/*pData->at(i).r = (std::uint8_t)255;
		pData->at(i).g = (std::uint8_t)255;
		pData->at(i).b = (std::uint8_t)255;*/

		/*float t = pData->at(i).y;
		pData->at(i).y = pData->at(i).z;
		pData->at(i).z = t;*/
	}

	PC_t::Ptr pOutput;
	dataManagement::CDepthInpaiting DepthInpainting;
	DepthInpainting.run(pData, pOutput);
	pcl::io::savePLYFileBinary<Point_t>("Output_Road_Seg_Sim.ply", *pOutput);
	*pOutput += *pData;
	pcl::io::savePLYFileBinary<Point_t>("Merge_Road_Seg_Sim.ply", *pOutput);

}