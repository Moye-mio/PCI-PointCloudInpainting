#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/CrossPlane.ply");

class TestExtremeValue : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}

	void generatePC(PC_t::Ptr& vData)
	{
		_ASSERTE(vData != NULL);

		for (float i = 0; i < 10; i += 0.5f)
		{
			vData->push_back(Point_t(i, 0, 0));

			if (i == 0) continue;

			vData->push_back(Point_t(0, i, 0));
			vData->push_back(Point_t(0, 0, i));
		}
	}
};

TEST_F(TestExtremeValue, NT_SameResultWhetherSetBox)
{
	std::string FileName = hiveUtility::hiveLocateFile(ModelPath);
	ASSERT_FALSE(FileName.empty());

	PC_t::Ptr pData(new PC_t);
	ASSERT_TRUE(pcl::io::loadPLYFile<Point_t>(ModelPath, *pData) != -1);
	ASSERT_TRUE(pData->size());

	core::CAABBEstimation Estimation(pData);
	core::SAABB Box = Estimation.compute();
	core::CExtremePoint HasBox(pData);
	ASSERT_TRUE(HasBox.compute());
	ASSERT_TRUE(HasBox.computeExtreme());
	std::vector<int> IndicesHasBox, ExtremeIndicesHasBox;
	HasBox.dumpIndices(IndicesHasBox);
	HasBox.dumpExtremeIndices(ExtremeIndicesHasBox);

	core::CExtremePoint NoBox(pData);
	ASSERT_TRUE(NoBox.compute());
	ASSERT_TRUE(NoBox.computeExtreme());
	std::vector<int> IndicesNoBox, ExtremeIndicesNoBox;
	NoBox.dumpIndices(IndicesNoBox);
	NoBox.dumpExtremeIndices(ExtremeIndicesNoBox);

	ASSERT_EQ(IndicesHasBox.size(), IndicesNoBox.size());
	for (int i = 0; i < IndicesHasBox.size(); i++)
		ASSERT_EQ(IndicesHasBox, IndicesNoBox);
	ASSERT_EQ(ExtremeIndicesHasBox.size(), ExtremeIndicesNoBox.size());
	for (int i = 0; i < ExtremeIndicesHasBox.size(); i++)
		ASSERT_EQ(ExtremeIndicesHasBox, ExtremeIndicesNoBox);
}

TEST_F(TestExtremeValue, NT_ComputeExtreme)
{
	PC_t::Ptr pData(new PC_t);
	generatePC(pData);
	ASSERT_TRUE(pData->size());

	core::CAABBEstimation Estimation(pData);
	core::SAABB Box = Estimation.compute();

	core::CExtremePoint EPE(pData);
	ASSERT_TRUE(EPE.compute());
	ASSERT_TRUE(EPE.computeExtreme());

	std::vector<int> Indices, ExtremeIndices;
	EPE.dumpIndices(Indices);
	ASSERT_EQ(Indices.size(), pData->size());
	EPE.dumpExtremeIndices(ExtremeIndices);
	ASSERT_EQ(ExtremeIndices.size(), 4);
}
