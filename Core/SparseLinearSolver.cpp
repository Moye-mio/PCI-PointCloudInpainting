#include "pch.h"
#include "SparseLinearSolver.h"

using namespace core;

CSparseLinearSolver::CSparseLinearSolver(const Eigen::MatrixXf& vA, const Eigen::MatrixXf& vB)
	: m_A(vA)
	, m_B(vB)
{ }

Eigen::MatrixXf CSparseLinearSolver::solve(int vMode /*= FullPivHouseholderQR*/)
{
	_ASSERTE(!__isZero(m_A));

	switch (vMode)
	{
	case PartialPivLU:
		_ASSERTE(__isInvertible(m_A));
		m_X = m_A.partialPivLu().solve(m_B);
		break;
	
	case FullPivLU:
		m_X = m_A.fullPivLu().solve(m_B);
		break;

	case HouseholderQR:
		m_X = m_A.householderQr().solve(m_B);
		break;

	case ColPivHouseholderQR:
		m_X = m_A.colPivHouseholderQr().solve(m_B);
		break;

	case FullPivHouseholderQR:
		m_X = m_A.fullPivHouseholderQr().solve(m_B);
		break;

	case LLT:
		_ASSERTE(__isPositiveDefinite(m_A));
		m_X = m_A.llt().solve(m_B);
		break;

	case LDLT:
		_ASSERTE(__isPositiveHalfDefinite(m_A) || __isNegativeHalfDefinite(m_A));
		m_X = m_A.ldlt().solve(m_B);
		break;

	default:
		break;
	}

	return m_X;
}

bool CSparseLinearSolver::__isInvertible(const Eigen::MatrixXf& vMat)
{
	// TODO

	return true;
}

bool CSparseLinearSolver::__isPositiveDefinite(const Eigen::MatrixXf& vMat)
{
	// TODO

	return true;
}

bool CSparseLinearSolver::__isPositiveHalfDefinite(const Eigen::MatrixXf& vMat)
{
	// TODO

	return true;
}

bool CSparseLinearSolver::__isNegativeHalfDefinite(const Eigen::MatrixXf& vMat)
{
	// TODO

	return true;
}

bool CSparseLinearSolver::__isZero(const Eigen::MatrixXf& vMat)
{
	return vMat == Eigen::MatrixXf::Zero(vMat.rows(), vMat.cols());
}
