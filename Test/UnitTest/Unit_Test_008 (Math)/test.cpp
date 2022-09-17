#include "pch.h"

class TestMath : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}
};

TEST_F(TestMath, DT_BilinearInterpolation)
{
	std::pair<float, float> Weights;
	std::vector<float> Values;
	Values.push_back(1);
	Values.push_back(2);
	Values.push_back(3);

	Weights = std::make_pair<float, float>(0.5, 0.5);
	ASSERT_DEATH(common::bilinearInterpolate(Weights, Values), ".*");
}

TEST_F(TestMath, NT_BilinearInterpolation)
{
	std::pair<float, float> Weights;
	std::vector<float> Values;
	Values.push_back(1);
	Values.push_back(2);
	Values.push_back(3);
	Values.push_back(4);

	Weights = std::make_pair<float, float>(0.5, 0.5);
	ASSERT_EQ(common::bilinearInterpolate(Weights, Values), 2.5);

	Weights = std::make_pair<float, float>(0, 0);
	ASSERT_EQ(common::bilinearInterpolate(Weights, Values), 1);

	Weights = std::make_pair<float, float>(0, 1);
	ASSERT_EQ(common::bilinearInterpolate(Weights, Values), 2);
}
