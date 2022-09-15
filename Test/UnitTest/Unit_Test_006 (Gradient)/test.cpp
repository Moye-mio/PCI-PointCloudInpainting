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
}

TEST_F(TestGradient, NT_SimpleMatrix)
{
	/*
	Coor:
	------------->
	|			 y(col, height)
	|
	|
	¡ýx(row, width)
	*/

	Eigen::Matrix<float, 3, 4> Mat;
	Mat << 0, 0, 0, 0,
		1, 1, 1, 1,
		2, 2, 2, 2;

	Eigen::Matrix<Eigen::Vector2f, 3, 4> GT;
	GT << Eigen::Vector2f(0.5, 0), Eigen::Vector2f(0.5, 0), Eigen::Vector2f(0.5, 0), Eigen::Vector2f(0.5, 0),
		Eigen::Vector2f(1, 0), Eigen::Vector2f(1, 0), Eigen::Vector2f(1, 0), Eigen::Vector2f(1, 0),
		Eigen::Vector2f(0.5, 0), Eigen::Vector2f(0.5, 0), Eigen::Vector2f(0.5, 0), Eigen::Vector2f(0.5, 0);

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

TEST_F(TestGradient, NT_HasEmptyValueMatrix)
{
	Eigen::Matrix<float, 3, 4> Mat;
	Mat << 0, 0, 0, 0,
		1, -FLT_MAX, -FLT_MAX, 1,
		2, 2, 2, 2;

	Eigen::Matrix<Eigen::Vector2f, 3, 4> GT;
	GT << Eigen::Vector2f(0.5, 0), m_Empty, m_Empty, Eigen::Vector2f(0.5, 0),
		m_Empty, m_Empty, m_Empty, m_Empty,
		Eigen::Vector2f(0.5, 0), m_Empty, m_Empty, Eigen::Vector2f(0.5, 0);

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