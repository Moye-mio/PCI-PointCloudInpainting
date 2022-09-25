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

	Eigen::MatrixXf Coeff, ConstNumbers;
	std::vector<Eigen::Vector2f> Unknowns;
	SolverBuilder.dumpMatrix(Coeff, ConstNumbers);
	SolverBuilder.dumpUnknowns(Unknowns);

	std::cout << "Coeff: \n" << Coeff << std::endl;
	std::cout << "Const: \n" << ConstNumbers << std::endl;
	for (int i = 0; i < Unknowns.size(); i++)
		std::cout << Unknowns[i] << ", ";
}