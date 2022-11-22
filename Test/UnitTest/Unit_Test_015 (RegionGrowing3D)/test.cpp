#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/CrossPlane.ply");

class TestRegionGrowing3D : public testing::Test
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

TEST_F(TestRegionGrowing3D, DT_InvalidData)
{
	PC_t::Ptr pData;
	core::CRegionGrowing3D Growing(pData);
	ASSERT_DEATH(Growing.Grow(0.5f), "");

	PC_t::Ptr pData2(new PC_t);
	core::CRegionGrowing3D Growing2(pData2);
	ASSERT_DEATH(Growing2.Grow(0.5f), "");
}

TEST_F(TestRegionGrowing3D, DT_NegativeRaduis)
{
	PC_t::Ptr pData(new PC_t);
	core::CRegionGrowing3D Growing(pData);
	ASSERT_DEATH(Growing.Grow(-1.0f), "");
}

TEST_F(TestRegionGrowing3D, NT_ManySingleCluster)
{
	PC_t::Ptr pData(new PC_t);
	generatePC(pData);

	std::vector<std::vector<int>> Result;
	core::CRegionGrowing3D Growing(pData);
	Growing.Grow(0.4f);
	Growing.dumpGrowingResult(Result);

	ASSERT_EQ(Result.size(), 58);
	ASSERT_EQ(Result[0].size(), 1);
}

