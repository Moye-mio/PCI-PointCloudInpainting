#pragma once

namespace core
{
	enum ESolverModes
	{
		PartialPivLU			= 0,		// 可逆		速度	++	精度	+
		FullPivLU				= 1,		//				-		+++
		HouseholderQR			= 2,		//				++		+
		ColPivHouseholderQR		= 3,		//				+		++
		FullPivHouseholderQR	= 4,		//				-		+++
		LLT						= 5,		// 正定			+++		+
		LDLT					= 6			// 正或负半定	+++		++
	};

	// solve equations like AX = B
	class CSparseLinearSolver
	{
	public:
		CSparseLinearSolver(const Eigen::MatrixXf& vA, const Eigen::MatrixXf& vB);
		~CSparseLinearSolver() = default;

		Eigen::MatrixXf solve(int vMode = FullPivHouseholderQR);

	private:
		bool __isInvertible(const Eigen::MatrixXf& vMat);
		bool __isPositiveDefinite(const Eigen::MatrixXf& vMat);
		bool __isPositiveHalfDefinite(const Eigen::MatrixXf& vMat);
		bool __isNegativeHalfDefinite(const Eigen::MatrixXf& vMat);
		bool __isZero(const Eigen::MatrixXf& vMat);

	private:
		Eigen::MatrixXf m_A;
		Eigen::MatrixXf m_B;
		Eigen::MatrixXf m_X;

	};
}