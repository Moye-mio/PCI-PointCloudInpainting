#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/CrossPlane.ply");

class TestGaussianDistribution : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}
};

TEST_F(TestGaussianDistribution, DT_EmptyData)
{
	std::vector<float> Data;
	core::CGaussianDistribution1D GD;
	ASSERT_DEATH(GD.setData(Data), "");
}

TEST_F(TestGaussianDistribution, DT_StepSetData)
{
	std::vector<float> Data;
	core::CGaussianDistribution1D GD;
	ASSERT_DEATH(GD.computeProbability(1.0f), "");
}

TEST_F(TestGaussianDistribution, DT_UnDefinedData)
{
	std::vector<float> Data;
	Data.push_back(1);
	core::CGaussianDistribution1D GD;
	float Pos;
	ASSERT_DEATH(GD.computeProbability(Pos), "");
	ASSERT_DEATH(GD.setChars(Pos, Pos), "");
}

TEST_F(TestGaussianDistribution, NT_OneData)
{
	std::vector<float> Data;
	Data.push_back(1);
	core::CGaussianDistribution1D GD;
	GD.setData(Data);
	ASSERT_EQ(GD.computeProbability(1), 1.0f);
	ASSERT_EQ(GD.computeProbability(0), 0.0f);

	std::vector<float> Data2;
	Data2.push_back(2);
	core::CGaussianDistribution2D GD2;
	GD2.setData(Data, Data2);
	ASSERT_EQ(GD2.computeProbability(1, 2), 1.0f);
	ASSERT_EQ(GD2.computeProbability(1, 1), 0.0f);
	ASSERT_EQ(GD2.computeProbability(2, 1), 0.0f);
}

TEST_F(TestGaussianDistribution, NT_NormalData)
{
	core::CGaussianDistribution1D GD;
	GD.setChars(3, 2);
	ASSERT_LT(GD.computeProbability(3) - 1 / std::sqrt(4 * std::numbers::pi), 0.000001);
	ASSERT_LT(GD.computeProbability(2) - 1 / (std::sqrt(4 * std::numbers::pi) * std::powf(std::numbers::e, 1/8)), 0.000001);
}