#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/CrossPlane.ply");

class TestBaseSurface : public testing::Test
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

TEST_F(TestBaseSurface, NT_CrossPlane)
{
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<dataManagement::IPCLoader>(hiveUtility::hiveGetFileSuffix(ModelPath));
	ASSERT_TRUE(pTileLoader);
	PC_t::Ptr pData = pTileLoader->loadDataFromFile(ModelPath);
	ASSERT_TRUE(pData);
}