#include "pch.h"

class TestUVGenerator : public testing::Test
{
protected:
	void SetUp() override
	{

	}

	void TearDown() override
	{
	}

	void generateTestData1(core::CHeightMap& voMask)
	{
		Eigen::Matrix<float, -1, -1> Mask;
		Mask.resize(4, 4);

		for (int i = 0; i < 4; i++)
			for (int k = 0; k < 4; k++)
			{
				if ((i == 1 || i == 2) && (k == 1 || k == 2))
					Mask.coeffRef(i, k) = 1;
				else
					Mask.coeffRef(i, k) = 0;
			}

		voMask = Mask;
	}
};

TEST_F(TestUVGenerator, DT_InValidInput)
{
	core::CSurfaceUVGenerator Generator;
	EXPECT_FALSE(Generator.generateUVSamples(core::CHeightMap(), 1));
	core::CHeightMap Raw;
	generateTestData1(Raw);
	EXPECT_FALSE(Generator.generateUVSamples(Raw, 0));
}

TEST_F(TestUVGenerator, NT_1)
{
	std::vector<Eigen::Vector2f> Data;
	core::CSurfaceUVGenerator Generator;
	core::CHeightMap Raw;
	generateTestData1(Raw);
	Generator.generateUVSamples(Raw, 4);
	Generator.dumpSamples(Data);

	for (const auto& e : Data)
	{
		EXPECT_TRUE(e[0] >= 0.25f && e[1] >= 0.25f && e[0] <= 0.75f && e[1] <= 0.75f);
	}
}