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