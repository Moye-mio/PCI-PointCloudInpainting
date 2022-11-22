#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/UniformSeg.ply");

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

TEST_F(TestExtractHoleBoundary, DT_InvalidData)
{
	PC_t::Ptr pData;
	core::CBorderExtractor Extractor(pData);
	ASSERT_DEATH(Extractor.Compute(0.5f), "");

	PC_t::Ptr pData2(new PC_t);
	core::CBorderExtractor Extractor2(pData2);
	ASSERT_DEATH(Extractor.Compute(0.5f), "");
}

TEST_F(TestExtractHoleBoundary, DT_NegativeRaduis)
{
	PC_t::Ptr pData(new PC_t);
	core::CBorderExtractor Extractor(pData);
	ASSERT_DEATH(Extractor.Compute(-1.0f), "");
}

TEST_F(TestExtractHoleBoundary, NT_UniformSeg)
{
	std::string FileName = hiveUtility::hiveLocateFile(ModelPath);
	ASSERT_FALSE(FileName.empty());

	PC_t::Ptr pData(new PC_t);
	ASSERT_TRUE(pcl::io::loadPLYFile<Point_t>(ModelPath, *pData) != -1);
	ASSERT_TRUE(pData->size());

	std::vector<int> Indices;
	core::CBorderExtractor Extractor(pData);
	Extractor.Compute(0.4);
	Extractor.dumpBorderIndices(Indices);

	PC_t::Ptr pCloud(new PC_t);
	for (auto e : Indices)
		pCloud->push_back(pData->at(e));

	ASSERT_EQ(pCloud->size(), 1170);
}