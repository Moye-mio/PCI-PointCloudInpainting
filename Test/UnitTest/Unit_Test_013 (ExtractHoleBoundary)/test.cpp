#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/CrossPlane.ply");

class TestExtractHoleBoundary : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}
};

TEST_F(TestExtractHoleBoundary, 1)
{
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<dataManagement::IPCLoader>(hiveUtility::hiveGetFileSuffix(ModelPath));
	ASSERT_TRUE(pTileLoader);
	PC_t::Ptr pData = pTileLoader->loadDataFromFile(ModelPath);
	ASSERT_TRUE(pData);

	core::CAABBEstimation Estimation(pData);
	core::SAABB Box = Estimation.compute();
	ASSERT_TRUE(Box.isValid());
	ASSERT_EQ(Box._Max, Eigen::Vector3f(10, 10, 5));
	ASSERT_EQ(Box._Min, Eigen::Vector3f(0, 0, 0));
}
