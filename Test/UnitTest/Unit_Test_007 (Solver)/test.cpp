#include "pch.h"

class TestSolver : public testing::Test
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


TEST_F(TestSolver, DT_ZeroMatrix)
{
	int MatrixSize = 10;
	Eigen::MatrixXf A, B, X;

	A = Eigen::MatrixXf::Zero(MatrixSize, MatrixSize);
	B = Eigen::MatrixXf::Zero(MatrixSize, 1);

	core::CSparseLinearSolver Solver(A, B);
	for (int i = 0; i < magic_enum::enum_count<core::ESolverModes>(); i++)
	{
		ASSERT_DEATH(Solver.solve(i), ".*");
	}
}

TEST_F(TestSolver, NT_Identity)
{
	int MatrixSize = 10;
	Eigen::MatrixXf A, B, X;

	A = Eigen::MatrixXf::Identity(MatrixSize, MatrixSize);
	B = Eigen::MatrixXf::Ones(MatrixSize, 1);

	core::CSparseLinearSolver Solver(A, B);
	for (int i = 0; i < magic_enum::enum_count<core::ESolverModes>(); i++)
	{
		X = Solver.solve(i);
		ASSERT_EQ(X, B);
	}
}

TEST_F(TestSolver, NT_SimpleMatrix1)
{
	Eigen::MatrixXf A(2, 2), X(2, 1), B(2, 1), GT(2, 1);

	A << -4, 1,
		1, -4;
	B << -3, -3;
	GT << 1, 1;

	core::CSparseLinearSolver Solver(A, B);
	for (int i = 0; i < 5; i++)
	{
		X = Solver.solve(i);
		for (int k = 0; k < 2; k++)
			ASSERT_LT(std::fabsf(X(k, 0) - GT(k, 0)), m_Epsilon);
	}
}

TEST_F(TestSolver, NT_SimpleMatrix2)
{
	Eigen::MatrixXf A(3, 3), X(3, 1), B(3, 1), GT(3, 1);

	A << -4, 1, 0,
		1, -4, 1,
		0, 1, -4;
	B << -3, -2, -3;
	GT << 1, 1, 1;

	core::CSparseLinearSolver Solver(A, B);
	for (int i = 0; i < 5; i++)
	{
		X = Solver.solve(i);
		for (int k = 0; k < 3; k++)
			ASSERT_LT(std::fabsf(X(k, 0) - GT(k, 0)), m_Epsilon);
	}
}
