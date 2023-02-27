#include "pch.h"

const std::string ModelPath1 = TESTMODEL_DIR + std::string("/Pyramid.ply");
const std::string ModelPath2 = TESTMODEL_DIR + std::string("/SlantPyramid.ply");
const std::string ModelPath3 = TESTMODEL_DIR + std::string("/KA_233_Simplified.las");
const std::string ModelPath4 = TESTMODEL_DIR + std::string("/HillRoad.las");
const std::string ModelPath5 = TESTMODEL_DIR + std::string("/Road_Seg_Sim.ply");
const std::string ModelPath6 = TESTMODEL_DIR + std::string("/Hill2.ply");
const std::string ModelPath7 = TESTMODEL_DIR + std::string("/GT/GT_CrossPlane.ply");
const std::string ModelPath8 = TESTMODEL_DIR + std::string("/SemiSphere.ply");

class TestSurface : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}

	void generatePC()
	{
		PC_t::Ptr pCloud(new PC_t);
		for (float i = 0; i < 5.0f; i += 0.1f)
			for (float k = 0; k < 10.05f; k += 0.1f)
				pCloud->push_back(Point_t(i, k, i, 255, 255, 255, 255));
		
		for (float i = 5; i < 10.05f; i += 0.1f)
			for (float k = 0; k < 10.05f; k += 0.1f)
				pCloud->push_back(Point_t(i, k, 10 - i, 255, 255, 255, 255));

		pcl::io::savePLYFileBinary("GT_CrossPlane.ply", *pCloud);
	}

	float m_Epsilon = 0.00001;
};

TEST_F(TestSurface, NT_)
{
	const auto ModelPath = ModelPath7;
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<dataManagement::IPCLoader>(hiveUtility::hiveGetFileSuffix(ModelPath));
	ASSERT_TRUE(pTileLoader);
	PC_t::Ptr pData = pTileLoader->loadDataFromFile(ModelPath);
	ASSERT_TRUE(pData);

	//for (int i = 0; i < pData->size(); i++)
	//{
	//	float t = pData->at(i).y;
	//	pData->at(i).y = pData->at(i).z;
	//	pData->at(i).z = t;
	//}

	PC_t::Ptr pOutput;
	dataManagement::CSurfaceConstruction Construction;
	Construction.run(pData, pOutput);
	//pcl::io::savePLYFileBinary<Point_t>("Output_Road_Seg_Sim.ply", *pOutput);
	//*pOutput += *pData;
	//pcl::io::savePLYFileBinary<Point_t>("Merge_Road_Seg_Sim.ply", *pOutput);

}