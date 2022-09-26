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

	void SolveEquations(const Eigen::MatrixXf& vMat, const Eigen::MatrixXf& vGT, Eigen::MatrixXf& voCoeff, Eigen::MatrixXf& voConstNumbers, std::vector<Eigen::Vector2f>& voUnknowns)
	{
		core::CHeightMap HeightMap, GTMap;
		ASSERT_TRUE(HeightMap.setHeightMap(vMat));
		ASSERT_TRUE(GTMap.setHeightMap(vGT));

		core::CGradientMap GradientMap, GoG;
		core::CGradientMapGenerator Generator;
		ASSERT_TRUE(Generator.generate(GTMap));
		Generator.dumpGradientMap(GradientMap);

		ASSERT_TRUE(Generator.generate(GradientMap));
		Generator.dumpGradientMap(GoG);

		dataManagement::CSolverBuilder SolverBuilder;
		SolverBuilder.setUp(HeightMap, GoG, GradientMap);
		SolverBuilder.dumpMatrix(voCoeff, voConstNumbers);
		SolverBuilder.dumpUnknowns(voUnknowns);
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

TEST_F(TestSetUpSolver, NT_InnerUnknowns)
{
	Eigen::Matrix<float, 3, 4> Mat, GT;
	Mat << 0, 0, 0, 0,
		1, -FLT_MAX, -FLT_MAX, 1,
		2, 2, 2, 2;

	GT << 0, 0, 0, 0,
		1, 1, 1, 1,
		2, 2, 2, 2;
	Eigen::MatrixXf Coeff, ConstNumbers, GTCoeff(2, 2), GTConst(2, 1);
	std::vector<Eigen::Vector2f> Unknowns, GTUnknowns;
	SolveEquations(Mat, GT, Coeff, ConstNumbers, Unknowns);

	GTCoeff << -4, 1, 1, -4;
	GTConst << -3, -3;
	GTUnknowns.push_back({1, 1});
	GTUnknowns.push_back({1, 2});
	ASSERT_EQ(Coeff, GTCoeff);
	ASSERT_EQ(ConstNumbers, GTConst);

	for (int i = 0; i < Unknowns.size(); i++)
		ASSERT_EQ(Unknowns[i], GTUnknowns[i]);
}

TEST_F(TestSetUpSolver, NT_AllUnknowns)
{
	Eigen::Matrix<float, 3, 4> Mat, GT;
	Mat << 0, 0, -FLT_MAX, -FLT_MAX,
		1, 1, -FLT_MAX, -FLT_MAX,
		2, 2, 2, 2;

	GT << 0, 0, 0, 0,
		1, 1, 1, 1,
		2, 2, 2, 2;
	Eigen::MatrixXf Coeff, ConstNumbers, GTCoeff(4, 4), GTConst(4, 1);
	std::vector<Eigen::Vector2f> Unknowns, GTUnknowns;
	SolveEquations(Mat, GT, Coeff, ConstNumbers, Unknowns);

	GTCoeff << -2, 1, 0, 0,
		1, -2, 0, 1,
		1, 0, -4, 1,
		0, 1, 0, -2;
	GTConst << 0, 1, -3, -2;
	GTUnknowns.push_back({ 0, 2 });
	GTUnknowns.push_back({ 0, 3 });
	GTUnknowns.push_back({ 1, 2 });
	GTUnknowns.push_back({ 1, 3 });
	ASSERT_EQ(Coeff, GTCoeff);
	ASSERT_EQ(ConstNumbers, GTConst);

	for (int i = 0; i < Unknowns.size(); i++)
		ASSERT_EQ(Unknowns[i], GTUnknowns[i]);
}