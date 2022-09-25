#include "pch.h"

class TestSetUpSolver : public testing::Test
{
protected:
	void SetUp() override
	{

	}

	void TearDown() override
	{
	}

	float m_Epsilon = 0.00001;
};

TEST_F(TestSetUpSolver, DT_MismatchingMaps)
{
	Eigen::Matrix<float, 3, 4> Mat;
	Mat << 0, 0, 0, 0,
		1, -FLT_MAX, -FLT_MAX, 1,
		2, 2, 2, 2;

	core::CHeightMap HeightMap;
	ASSERT_TRUE(HeightMap.setHeightMap(Mat));

	core::CGradientMap GradientMap, GoG;
	core::CGradientMapGenerator Generator;
	ASSERT_TRUE(Generator.generate(HeightMap));
	Generator.dumpGradientMap(GradientMap);

	ASSERT_TRUE(Generator.generate(GradientMap));
	Generator.dumpGradientMap(GoG);

	dataManagement::CSolverBuilder SolverBuilder;
	ASSERT_DEATH(SolverBuilder.setUp(HeightMap, GoG, GradientMap), ".*");
}

TEST_F(TestSetUpSolver, NT)
{
	Eigen::Matrix<float, 3, 4> Mat, GT;
	Mat << 0, 0, 0, 0,
		1, -FLT_MAX, -FLT_MAX, 1,
		2, 2, 2, 2;

	GT << 0, 0, 0, 0,
		1, 1, 1, 1,
		2, 2, 2, 2;

	core::CHeightMap HeightMap, GTMap;
	ASSERT_TRUE(HeightMap.setHeightMap(Mat));
	ASSERT_TRUE(GTMap.setHeightMap(GT));

	core::CGradientMap GradientMap, GoG;
	core::CGradientMapGenerator Generator;
	ASSERT_TRUE(Generator.generate(GTMap));
	Generator.dumpGradientMap(GradientMap);

	ASSERT_TRUE(Generator.generate(GradientMap));
	Generator.dumpGradientMap(GoG);

	for (int i = 0; i < GoG.getWidth(); i++)
		for (int k = 0; k < GoG.getHeight(); k++)
			if (!GoG.isEmptyValue(i, k))
				ASSERT_EQ(GoG.getValueAt(i, k), Eigen::Vector2f(0, 0));
	
	dataManagement::CSolverBuilder SolverBuilder;
	SolverBuilder.setUp(HeightMap, GoG, GradientMap);

	Eigen::MatrixXf Coeff, ConstNumbers, GTCoeff(2, 2), GTConst(2, 1);
	std::vector<Eigen::Vector2f> Unknowns, GTUnknowns;
	SolverBuilder.dumpMatrix(Coeff, ConstNumbers);
	SolverBuilder.dumpUnknowns(Unknowns);
	GTCoeff << -4, 1, 1, -4;
	GTConst << -3, -3;
	GTUnknowns.push_back({1, 1});
	GTUnknowns.push_back({1, 2});
	ASSERT_EQ(Coeff, GTCoeff);
	ASSERT_EQ(ConstNumbers, GTConst);

	for (int i = 0; i < Unknowns.size(); i++)
		ASSERT_EQ(Unknowns[i], GTUnknowns[i]);
}