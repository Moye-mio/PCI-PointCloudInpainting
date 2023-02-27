#include "pch.h"

#include "BSplineSurface.h"
#include "BSplineCurve.h"

using namespace core;

IBSplineSurface::IBSplineSurface(int vDegree, bool vIsClamped /*= true*/)
	: m_IsClamped(vIsClamped)
{
	_ASSERTE(vDegree == 3);
	m_Degree = vDegree;
}

bool IBSplineSurface::setControlPoints(const Eigen::Matrix<SPoint, -1, -1>& vPoints)
{
	_ASSERTE(vPoints.data());
	_ASSERTE(vPoints.rows() >= m_Degree + 1 && vPoints.cols() >= m_Degree + 1);

	m_ControlPoints = vPoints;
	return true;
}

SPoint IBSplineSurface::sample(float vU, float vV)
{
	_ASSERTE(vU >= 0 && vU <= 1);
	_ASSERTE(m_ControlPoints.rows() >= m_Degree + 1 && m_ControlPoints.cols() >= m_Degree + 1);
	_ASSERTE(m_Degree == 3);

	return __sample(m_ControlPoints, vU, vV);
}

SPoint IBSplineSurface::__sample(const Eigen::Matrix<SPoint, -1, -1>& vPoints, float vU, float vV)
{
	std::vector<SPoint> CPInRow;
	for (int i = 0; i < vPoints.cols(); i++)
	{
		std::vector<SPoint> CPPerCol;
		for (int k = 0; k < vPoints.rows(); k++)
			CPPerCol.emplace_back(vPoints.coeff(k, i));
		core::CBSplineCurve Curve(m_Degree);
		Curve.setControlPoints(CPPerCol);
		SPoint r = Curve.sample(vU);
		CPInRow.emplace_back(r);
	}

	core::CBSplineCurve Curve(m_Degree);
	Curve.setControlPoints(CPInRow);
	SPoint Result = Curve.sample(vV);
	return Result;
}
