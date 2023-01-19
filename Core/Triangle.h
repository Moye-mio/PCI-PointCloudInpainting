#pragma once
#include "Plane.h"

namespace core
{
	class CTriangle
	{
	public:
		CTriangle() = default;
		CTriangle(const SPoint& vPoint1, const SPoint& vPoint2, const SPoint& vPoint3);
		
		SPoint& operator[](unsigned int i);
		const SPoint& operator[](unsigned int i) const;
		const unsigned int size() const;

		bool calcPlane(common::SPlane& voPlane, bool vIsNorm = true) const;
		bool calcBaryCoor(const core::SPoint& vPoint, Eigen::Vector3f& vCoor) const;
		[[nodiscard]] bool isRayIntersection(const core::SPoint& vPoint, const Eigen::Vector3f& vRayDir) const;
		[[nodiscard]] bool isValid() const;
		
	private:
		bool __isValid() const;

	private:
		SPoint m_P1;
		SPoint m_P2;
		SPoint m_P3;
	};
}
