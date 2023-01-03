#include "pch.h"
#include "Triangle.h"

using namespace core;

CTriangle::CTriangle(const SPoint& vPoint1, const SPoint& vPoint2, const SPoint& vPoint3)
	: m_P1(vPoint1)
	, m_P2(vPoint2)
	, m_P3(vPoint3)
{}

core::SPoint& CTriangle::operator[](unsigned int i)
{
	_ASSERTE(i < size());
	switch (i)
	{
	case 0:
		return m_P1;
		break;
	case 1:
		return m_P2;
		break;
	case 2:
		return m_P3;
		break;
	}
}

const core::SPoint& CTriangle::operator[](unsigned int i) const
{
	_ASSERTE(i < size());
	switch (i)
	{
	case 0:
		return m_P1;
		break;
	case 1:
		return m_P2;
		break;
	case 2:
		return m_P3;
		break;
	}
}

const unsigned int CTriangle::size() const
{
	return 3;
}

void CTriangle::calcPlane(common::SPlane& voPlane, bool vIsNorm /* = true */) const
{
	_ASSERTE(__isValid());

	Eigen::Vector3f Edge1 = m_P1 - m_P2;
	Eigen::Vector3f Edge2 = m_P1 - m_P3;
	Eigen::Vector3f Normal = Edge1.cross(Edge2);
	float D = -Normal.dot(m_P1);

	voPlane = common::SPlane(Normal[0], Normal[1], Normal[2], D);
	_ASSERTE(voPlane.isValid());
	if (vIsNorm)
		voPlane.normalize();
}

bool CTriangle::__isValid() const
{
	if (!(m_P1.isValid() && m_P2.isValid() && m_P3.isValid()))
		return false;
	if (m_P1 == m_P2 || m_P2 == m_P3 || m_P1 == m_P3 || m_P1 - m_P2 == m_P1 - m_P3)
		return false;
	return true;
}

/* MT Method */
bool CTriangle::isRayIntersection(const core::SPoint& vPoint, const Eigen::Vector3f& vRayDir) const
{
	_ASSERTE(vPoint.isValid());
	_ASSERTE(__isValid());

	Eigen::Vector3f E1 = m_P2 - m_P1;
	Eigen::Vector3f E2 = m_P3 - m_P1;
	Eigen::Vector3f S = vPoint - m_P1;
	Eigen::Vector3f S1 = vRayDir.cross(E2);
	Eigen::Vector3f S2 = S.cross(E1);

	float B1 = S1.dot(S) / S1.dot(E1);
	float B2 = S2.dot(vRayDir) / S1.dot(E1);

	std::cout << "S1.dot(E1): " << S1.dot(E1) << std::endl;

	if (B1 >= 0 && B2 >= 0 && B1 + B2 <= 1)
		return true;
	else
		return false;
}

void CTriangle::calcBaryCoor(const core::SPoint& vPoint, Eigen::Vector3f& vCoor) const
{
	_ASSERTE(vPoint.isValid());
	_ASSERTE(__isValid());

	float A1 = 0.5f * (vPoint - m_P2).cross(vPoint - m_P3).norm();
	float A2 = 0.5f * (vPoint - m_P1).cross(vPoint - m_P3).norm();
	float A3 = 0.5f * (vPoint - m_P1).cross(vPoint - m_P2).norm();
	float A = 0.5f * (m_P3 - m_P1).cross(m_P3 - m_P2).norm();
	_ASSERTE(A > 0);

	float Epsilon = 0.00001f;
	_ASSERTE(A - (A1 + A2 + A3) < Epsilon);

	vCoor = Eigen::Vector3f(A1 / A, A2 / A, A3 / A);
}

bool CTriangle::isValid() const
{
	return __isValid();
}

