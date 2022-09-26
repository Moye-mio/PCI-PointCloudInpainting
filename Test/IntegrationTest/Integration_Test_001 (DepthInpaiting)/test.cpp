#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/Pyramid.ply");

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
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<dataManagement::IPCLoader>(hiveUtility::hiveGetFileSuffix(ModelPath));	/* 14175 */
	ASSERT_TRUE(pTileLoader);
	PC_t::Ptr pData = pTileLoader->loadDataFromFile(ModelPath);
	ASSERT_TRUE(pData);
	PC_t::Ptr pOutput;
	dataManagement::CDepthInpaiting DepthInpainting;
	DepthInpainting.run(pData, pOutput);
	pcl::io::savePLYFileBinary<Point_t>("Output.ply", *pOutput);
	*pOutput += *pData;
	pcl::io::savePLYFileBinary<Point_t>("Merge.ply", *pOutput);

}