#include "pch.h"

#include "BSplineCurve.h"

using namespace core;

CBSplineCurve::CBSplineCurve(int vDegree, bool vIsClamped)
	: m_IsClamped(vIsClamped)
{
	_ASSERTE(vDegree >= 1);
	m_Degree = vDegree;
}

bool CBSplineCurve::setControlPoints(const std::vector<SPoint>& vPoints)
{
	_ASSERTE(vPoints.size() >= m_Degree + 1);
	if (m_IsClamped)
	{
		for (int i = 0; i < m_Degree - 1; i++)
			m_ControlPoints.emplace_back(vPoints[0]);
		for (const auto& e : vPoints)
			m_ControlPoints.emplace_back(e);
		for (int i = 0; i < m_Degree - 1; i++)
			m_ControlPoints.emplace_back(vPoints.back());
	}
	else
		m_ControlPoints = vPoints;
	return true;
}

SPoint CBSplineCurve::sample(float vU)
{
	_ASSERTE(vU >= 0 && vU <= 1);
	int NodeNumber = m_ControlPoints.size() - m_Degree;
	_ASSERTE(NodeNumber >= 1);
	_ASSERTE(m_Degree == 3);

	int StartControlPoint = (vU != 1.0f) ? (int)(vU * NodeNumber) : NodeNumber - 1;
	float Para = vU * NodeNumber - StartControlPoint;

	float Coef[4];
	Coef[0] = (-Para * Para * Para + 3 * Para * Para - 3 * Para + 1.0f) / 6.0f;
	Coef[1] = (3 * Para * Para * Para - 6 * Para * Para + 4.0f) / 6.0f;
	Coef[2] = (-3 * Para * Para * Para + 3 * Para * Para + 3 * Para + 1.0f) / 6.0f;
	Coef[3] = (Para * Para * Para) / 6.0f;

	SPoint Result(0.0f, 0.0f, 0.0f);
	for (int i = 0; i < 3; i++)
		for (int k = 0; k < m_Degree + 1; k++)
			Result[i] += Coef[k] * m_ControlPoints[StartControlPoint + k][i];

	return Result;
}
