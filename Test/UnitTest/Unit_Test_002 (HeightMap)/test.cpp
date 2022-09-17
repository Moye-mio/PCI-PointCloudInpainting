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

	float m_Epsilon = 0.00001;
};

TEST_F(TestHeightMap, DT_NullCloud)
{
	PC_t::Ptr pCloud;
	core::CHeightMapGenerator Generator;
	ASSERT_DEATH(Generator.setCloud(pCloud), ".*");
}

TEST_F(TestHeightMap, DT_EmptyCloud)
{
	PC_t::Ptr pCloud(new PC_t);
	core::CHeightMapGenerator Generator;
	ASSERT_DEATH(Generator.setCloud(pCloud), ".*");
}

TEST_F(TestHeightMap, DT_IncorrectMapSize)
{
	core::CHeightMapGenerator Generator;
	ASSERT_DEATH(Generator.generate(0, 0), ".*");
}

TEST_F(TestHeightMap, DT_SkipSetCloud)
{
	core::CHeightMapGenerator Generator;
	ASSERT_DEATH(Generator.generate(1, 1), ".*");
}

TEST_F(TestHeightMap, NT_1)
{
	PC_t::Ptr pCloud = getCloud();
	core::CHeightMap Map;
	core::CHeightMapGenerator Generator;
	Generator.setCloud(pCloud);
	Generator.generate(1, 1);
	Generator.dumpHeightMap(Map);

	_ASSERTE(Map.getValueAt(0, 0) == 8);
}

TEST_F(TestHeightMap, NT_2)
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

TEST_F(TestHeightMap, NT_3)
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

TEST_F(TestHeightMap, NT_Sample)
{
	core::CHeightMap Map;
	Eigen::MatrixXf Mat(3, 3);
	Mat << 0, 0, 0,
		1, 1, 1,
		2, 2, 2;

	Map.setHeightMap(Mat);
	ASSERT_TRUE(Map.isValid());

	ASSERT_EQ(Map.Sample(std::make_pair<float, float>(0, 0)), 0);
	ASSERT_EQ(Map.Sample(std::make_pair<float, float>(0.4, 2.3)), 0);
	ASSERT_LT(Map.Sample(std::make_pair<float, float>(2.3, 0.4)) - 1.8, m_Epsilon);
	ASSERT_EQ(Map.Sample(std::make_pair<float, float>(0.5, 0.5)), 0);
	ASSERT_EQ(Map.Sample(std::make_pair<float, float>(1, 1)), 0.5);
	ASSERT_EQ(Map.Sample(std::make_pair<float, float>(1.5, 1.5)), 1);
	ASSERT_EQ(Map.Sample(std::make_pair<float, float>(2, 2)), 1.5);
	ASSERT_EQ(Map.Sample(std::make_pair<float, float>(2.5, 2.5)), 2);
	ASSERT_EQ(Map.Sample(std::make_pair<float, float>(2.9, 2.9)), 2);
	ASSERT_LT(Map.Sample(std::make_pair<float, float>(1.9, 1.9)) - 1.4, m_Epsilon);
	ASSERT_LT(Map.Sample(std::make_pair<float, float>(1.6, 1.9)) - 1.1, m_Epsilon);
}