#include "pch.h"

class TestNormalSampler : public testing::Test
{
protected:
	void SetUp() override
	{

	}

	void TearDown() override
	{
	}
};

TEST_F(TestNormalSampler, DT_InValidInput)
{
	core::CNormalSampler Sampler;
	EXPECT_FALSE(Sampler.setData(Eigen::Matrix<core::SVertex, -1, -1>()));
}

TEST_F(TestNormalSampler, NT_PlaneNormalSample)
{
	Eigen::Matrix<core::SVertex, -1, -1> Data;
	Data.resize(10, 10);
	for (int i = 0; i < 10; i++)
		for (int k = 0; k < 10; k++)
			Data.coeffRef(i, k) = core::SVertex(i, k, 0, i * 0.1f, k * 0.1f);

	core::CNormalSampler Sampler;
	Sampler.setData(Data);

	auto Set = hiveMath::hiveGenerateRandomRealSet(0.0f, 1.0f, 20);
	
	for (int i = 0; i < Set.size(); i += 2)
	{
		auto r = Sampler.sample(Eigen::Vector2f(Set[i], Set[i + 1]));
		Eigen::Vector3f Normal = r.value();
		EXPECT_TRUE(Normal == Eigen::Vector3f(0, 0, 1));
	}
}

TEST_F(TestNormalSampler, NT_InclineNormalSample)
{
	Eigen::Matrix<core::SVertex, -1, -1> Data;
	Data.resize(11, 11);
	for (int i = 0; i < 11; i++)
		for (int k = 0; k < 11; k++)
		{
			if (i <= 5)
				Data.coeffRef(i, k) = core::SVertex(i, k, i, i * 0.1f, k * 0.1f);
			else
				Data.coeffRef(i, k) = core::SVertex(i, k, 10 - i, i * 0.1f, k * 0.1f);
		}

	core::CNormalSampler Sampler;
	Sampler.setData(Data);

	auto Set = hiveMath::hiveGenerateRandomRealSet(0.0f, 1.0f, 20);
	for (int i = 0; i < Set.size(); i += 2)
	{
		auto r = Sampler.sample(Eigen::Vector2f(Set[i], Set[i + 1]));
		Eigen::Vector3f Normal = r.value();
		/*EXPECT_TRUE(Normal == Eigen::Vector3f(0, 0, 1));*/
		std::cout << "(" << Normal[0] << ", " << Normal[1] << ", " << Normal[2] << ")" << std::endl;
	}
}
