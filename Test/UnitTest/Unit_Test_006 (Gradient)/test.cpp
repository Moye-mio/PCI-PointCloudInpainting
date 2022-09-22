#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/CrossPlane.ply");

class TestGradient : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}

	Eigen::Vector2f m_Empty = { -FLT_MAX, -FLT_MAX };
};

TEST_F(TestGradient, DT_InValidMap)
{
	core::CHeightMap Map;
	core::CGradientMapGenerator Generator;
	ASSERT_DEATH(Generator.generate(Map), ".*");

	core::CGradientMap GMap;
	ASSERT_DEATH(Generator.generate(GMap), ".*");
}

/*
	Coor:
	------------->
	|			 y(col, height)
	|
	|
	¡ýx(row, width)
	*/

TEST_F(TestGradient, NT_SimpleMatrix)
{
	Eigen::Matrix<float, 3, 4> Mat1;
	Mat1 << 0, 0, 0, 0,
		1, 1, 1, 1,
		2, 2, 2, 2;

	Eigen::Matrix<float, 3, 4> Mat2;
	Mat2 << 0, 0, 0, 0,
		0, 0, 0, 0,
		2, 2, 2, 2;

	std::vector<Eigen::MatrixXf> Mats;
	Mats.emplace_back(Mat1);
	Mats.emplace_back(Mat2);

	std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> GT;
	GT.push_back(std::make_pair(Eigen::Vector2f(1.0f, 0.0f), Eigen::Vector2f(0.0f, 0.0f)));
	GT.push_back(std::make_pair(Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(1.0f, 0.0f)));

	for (int Index = 0; Index < Mats.size(); Index++)
	{
		core::CHeightMap HeightMap;
		ASSERT_TRUE(HeightMap.setHeightMap(Mats[Index]));

		core::CGradientMap GradientMap;
		core::CGradientMapGenerator Generator;
		ASSERT_TRUE(Generator.generate(HeightMap));
		Generator.dumpGradientMap(GradientMap);

		for (int i = 0; i < GradientMap.getWidth(); i++)
			for (int k = 0; k < GradientMap.getHeight(); k++)
				ASSERT_EQ(GradientMap.getValueAt(i, k), GT[Index].first + Index * Eigen::Vector2f(i, 0.0f));

		ASSERT_TRUE(Generator.generate(GradientMap));
		Generator.dumpGradientMap(GradientMap);

		for (int i = 0; i < GradientMap.getWidth(); i++)
			for (int k = 0; k < GradientMap.getHeight(); k++)
				ASSERT_EQ(GradientMap.getValueAt(i, k), GT[Index].second);
	}
}

TEST_F(TestGradient, NT_HasEmptyValueMatrix)
{
	Eigen::Matrix<float, 3, 4> Mat;
	Mat << 0, 0, 0, 0,
		1, -FLT_MAX, -FLT_MAX, 1,
		2, 2, 2, 2;

	Eigen::Matrix<Eigen::Vector2f, 3, 4> GT;
	GT << Eigen::Vector2f(1, 0), m_Empty, m_Empty, Eigen::Vector2f(1, 0),
		m_Empty, m_Empty, m_Empty, m_Empty,
		Eigen::Vector2f(1, 0), m_Empty, m_Empty, Eigen::Vector2f(1, 0);

	core::CHeightMap HeightMap;
	ASSERT_TRUE(HeightMap.setHeightMap(Mat));

	core::CGradientMap GradientMap;
	core::CGradientMapGenerator Generator;
	ASSERT_TRUE(Generator.generate(HeightMap));
	Generator.dumpGradientMap(GradientMap);

	for (int i = 0; i < GradientMap.getWidth(); i++)
		for (int k = 0; k < GradientMap.getHeight(); k++)
			ASSERT_EQ(GradientMap.getValueAt(i, k), GT(i, k));
}