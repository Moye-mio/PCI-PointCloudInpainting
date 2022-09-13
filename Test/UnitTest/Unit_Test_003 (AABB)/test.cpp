#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/CrossPlane.ply");

class TestAABB : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}
};

TEST_F(TestAABB, DT_NullCloud)
{
	PC_t::Ptr pCloud;
	core::CAABBEstimation Estimation(pCloud);
	ASSERT_DEATH(Estimation.compute(), ".*");
}

TEST_F(TestAABB, DT_EmptyCloud)
{
	PC_t::Ptr pCloud(new PC_t);
	core::CAABBEstimation Estimation(pCloud);
	ASSERT_DEATH(Estimation.compute(), ".*");
}

TEST_F(TestAABB, NT_SinglePoint)
{
	PC_t::Ptr pCloud(new PC_t);
	pCloud->push_back(Point_t(0, 0, 0));

	core::CAABBEstimation Estimation(pCloud);
	core::SAABB Box = Estimation.compute();
	ASSERT_TRUE(Box.isValid());
	ASSERT_EQ(Box._Max, Eigen::Vector3f(0, 0, 0));
	ASSERT_EQ(Box._Min, Eigen::Vector3f(0, 0, 0));
}

TEST_F(TestAABB, NT_Model)
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
