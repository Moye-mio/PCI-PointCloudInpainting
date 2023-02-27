#pragma once
#include "Plane.h"

namespace core
{
	class CTriangle
	{
	public:
		CTriangle() = default;
		CTriangle(const Eigen::Vector3f& vPoint1, const Eigen::Vector3f& vPoint2, const Eigen::Vector3f& vPoint3);
		
		Eigen::Vector3f& operator[](unsigned int i);
		const Eigen::Vector3f& operator[](unsigned int i) const;
		const unsigned int size() const;

		bool calcPlane(common::SPlane& voPlane, bool vIsNorm = true) const;
		bool calcBaryCoor(const Eigen::Vector3f& vPoint, Eigen::Vector3f& vCoor) const;
		[[nodiscard]] bool isRayIntersection(const Eigen::Vector3f& vPoint, const Eigen::Vector3f& vRayDir) const;
		[[nodiscard]] bool isValid() const;
		
	private:
		bool __isValid() const;
		bool __isPointValid() const;

	private:
		Eigen::Vector3f m_P1;
		Eigen::Vector3f m_P2;
		Eigen::Vector3f m_P3;
	};
}
