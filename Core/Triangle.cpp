#include "pch.h"
#include "Triangle.h"

using namespace core;

CTriangle::CTriangle(const Eigen::Vector3f& vPoint1, const Eigen::Vector3f& vPoint2, const Eigen::Vector3f& vPoint3)
	: m_P1(vPoint1)
	, m_P2(vPoint2)
	, m_P3(vPoint3)
{}

Eigen::Vector3f& CTriangle::operator[](unsigned int i)
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

const Eigen::Vector3f& CTriangle::operator[](unsigned int i) const
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

bool CTriangle::calcPlane(common::SPlane& voPlane, bool vIsNorm /* = true */) const
{
	if (__isValid() == false)
	{
		std::cout << "ERROR: Triangle is not Valid..." << std::endl;
		return false;
	}
	//_HIVE_EARLY_RETURN(__isValid() == false, "ERROR: Triangle is not Valid...", false);

	Eigen::Vector3f Edge1 = m_P1 - m_P2;
	Eigen::Vector3f Edge2 = m_P1 - m_P3;
	Eigen::Vector3f Normal = Edge1.cross(Edge2);
	float D = -Normal.dot(m_P1);

	voPlane = common::SPlane(Normal[0], Normal[1], Normal[2], D);
	_HIVE_EARLY_RETURN(voPlane.isValid() == false, "ERROR: Plane is not Valid...", false);
	if (vIsNorm)
		voPlane.normalize();
	return true;
}

bool CTriangle::__isValid() const
{
	if (__isPointValid() == false)
		return false;
	if (m_P1 == m_P2 || m_P2 == m_P3 || m_P1 == m_P3 || m_P1 - m_P2 == m_P1 - m_P3)
		return false;
	return true;
}

/* MT Method */
bool CTriangle::isRayIntersection(const Eigen::Vector3f& vPoint, const Eigen::Vector3f& vRayDir) const
{
	_HIVE_EARLY_RETURN(__isPointValid() == false, "ERROR: Point in Ray Intersection is not Valid...", false);
	_HIVE_EARLY_RETURN(__isValid() == false, "ERROR: Triangle in Ray Intersection is not Valid...", false);

	Eigen::Vector3f E1 = m_P2 - m_P1;
	Eigen::Vector3f E2 = m_P3 - m_P1;
	Eigen::Vector3f S = vPoint - m_P1;
	Eigen::Vector3f S1 = vRayDir.cross(E2);
	Eigen::Vector3f S2 = S.cross(E1);

	float B1 = S1.dot(S) / S1.dot(E1);
	float B2 = S2.dot(vRayDir) / S1.dot(E1);

	//std::cout << "S1.dot(E1): " << S1.dot(E1) << std::endl;

	if (B1 >= 0 && B2 >= 0 && B1 + B2 <= 1)
		return true;
	else
		return false;
}

bool CTriangle::calcBaryCoor(const Eigen::Vector3f& vPoint, Eigen::Vector3f& voCoor) const
{
	_HIVE_EARLY_RETURN(__isPointValid() == false, "ERROR: Point in Calc BaryCoor is not Valid...", false);
	_HIVE_EARLY_RETURN(__isValid() == false, "ERROR: Triangle in Calc BaryCoor is not Valid...", false);

	float A1 = 0.5f * (vPoint - m_P2).cross(vPoint - m_P3).norm();
	float A2 = 0.5f * (vPoint - m_P1).cross(vPoint - m_P3).norm();
	float A3 = 0.5f * (vPoint - m_P1).cross(vPoint - m_P2).norm();
	float A = 0.5f * (m_P3 - m_P1).cross(m_P3 - m_P2).norm();
	_HIVE_EARLY_RETURN(A <= 0, "ERROR: Triangle Area in Calc BaryCoor is not Valid...", false);

	float Epsilon = 0.00001f;
	if (A < Epsilon * 100) Epsilon /= 100;
	_HIVE_EARLY_RETURN(std::fabsf(A - (A1 + A2 + A3)) >= Epsilon, "ERROR: Area Rate in Calc BaryCoor is not Valid...", false);

	hiveEventLogger::hiveOutputEvent(_FORMAT_STR4("Area: %1%, %2%, %3%, %4%", A1, A2, A3, A));
	voCoor = Eigen::Vector3f(A1 / (A1 + A2 + A3), A2 / (A1 + A2 + A3), A3 / (A1 + A2 + A3));
	return true;
}

bool CTriangle::isValid() const
{
	return __isValid();
}

bool CTriangle::__isPointValid() const
{
	std::vector<Eigen::Vector3f> Points;
	for (const auto& e : Points)
		if (std::isnan(e.x()) || std::isnan(e.y()) || std::isnan(e.z()))
			return false;
	return true;
}