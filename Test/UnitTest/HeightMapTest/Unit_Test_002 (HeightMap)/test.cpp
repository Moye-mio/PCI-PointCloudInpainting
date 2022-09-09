#include "pch.h"

class TestHeightMap : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}

	PC_t::Ptr getCloud()
	{
		PC_t::Ptr pCloud(new PC_t);
		pCloud->push_back(Point_t(0, 0, 0, 0, 0, 0, 0));
		pCloud->push_back(Point_t(0, 1, 1, 0, 0, 0, 0));
		pCloud->push_back(Point_t(0, 2, 2, 0, 0, 0, 0));
		pCloud->push_back(Point_t(1, 0, 3, 0, 0, 0, 0));
		pCloud->push_back(Point_t(1, 1, 4, 0, 0, 0, 0));
		pCloud->push_back(Point_t(1, 2, 5, 0, 0, 0, 0));
		pCloud->push_back(Point_t(2, 0, 6, 0, 0, 0, 0));
		pCloud->push_back(Point_t(2, 1, 7, 0, 0, 0, 0));
		pCloud->push_back(Point_t(2, 2, 8, 0, 0, 0, 0));
		return pCloud;
	}
};

TEST_F(TestHeightMap, 2)
{
	PC_t::Ptr pCloud = getCloud();
	core::CHeightMap Map;
	core::CHeightMapGenerator Generator;
	Generator.setCloud(pCloud);
	Generator.generate(2, 2);
	Generator.dumpHeightMap(Map);

	_ASSERTE(Map.getValueAt(0, 0) == 0);
	_ASSERTE(Map.getValueAt(0, 1) == 2);
	_ASSERTE(Map.getValueAt(1, 0) == 6);
	_ASSERTE(Map.getValueAt(1, 1) == 8);
}

TEST_F(TestHeightMap, 3)
{
	PC_t::Ptr pCloud = getCloud();
	core::CHeightMap Map;
	core::CHeightMapGenerator Generator;
	Generator.setCloud(pCloud);
	Generator.generate(3, 3);
	Generator.dumpHeightMap(Map);

	for (int i = 0; i < Map.getWidth(); i++)
		for (int k = 0; k < Map.getHeight(); k++)
			_ASSERTE(Map.getValueAt(i, k) == k + i * 3);
}