#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/CrossPlane.ply");
const std::string SavePath = TESTMODEL_DIR + std::string("/Save.ply");

class TestIO : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}

	bool isPointEqual(const Point_t& vLhs, const Point_t& vRhs)
	{
		return (vLhs.getVector3fMap() == vRhs.getVector3fMap() && vLhs.rgba == vRhs.rgba);
	}
};

TEST_F(TestIO, Load)
{
	std::string LowerFileName = hiveUtility::hiveLocateFile(ModelPath);
	ASSERT_FALSE(LowerFileName.empty());

	auto Suffix = hiveUtility::hiveGetFileSuffix(ModelPath);
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<dataManagement::IPCLoader>(Suffix);
	ASSERT_TRUE(pTileLoader);

	PC_t::Ptr pData = pTileLoader->loadDataFromFile(ModelPath);
	ASSERT_TRUE(pData);
	EXPECT_EQ(pData->size(), 14275);
}

TEST_F(TestIO, Save)
{
	PC_t::Ptr pCloud(new PC_t);
	pCloud->push_back(Point_t(0, 0, 0, 0, 0, 0, 0));
	pCloud->push_back(Point_t(1, 1, 1, 1, 1, 1, 1));
	pCloud->push_back(Point_t(2, 2, 2, 2, 2, 2, 2));

	auto* pSaver = hiveDesignPattern::hiveGetOrCreateProduct<dataManagement::IPCSaver>(hiveUtility::hiveGetFileSuffix(SavePath) + "_Save");
	ASSERT_TRUE(pSaver);

	pSaver->saveDataToFileV(*pCloud, SavePath);

	std::string LowerFileName = hiveUtility::hiveLocateFile(SavePath);
	ASSERT_FALSE(LowerFileName.empty());

	auto Suffix = hiveUtility::hiveGetFileSuffix(SavePath);
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<dataManagement::IPCLoader>(Suffix);
	ASSERT_TRUE(pTileLoader);

	PC_t::Ptr pData = pTileLoader->loadDataFromFile(SavePath);
	ASSERT_TRUE(pData);

	EXPECT_EQ(pData->size(), pCloud->size());
	for (auto & e : *pData)
	{
		int i = 0;
		for (; i < pCloud->size(); i++)
			if (isPointEqual(e, pCloud->at(i))) break;
		ASSERT_FALSE(i == pCloud->size());
	}
	
}