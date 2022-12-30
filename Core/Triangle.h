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
		const unsigned int size() const;

		void calcPlane(common::SPlane& voPlane);
		[[nodiscard]] bool isRayIntersection(const core::SPoint& vPoint, const Eigen::Vector3f& vRayDir);
		
	private:
		bool __isValid();

	private:
		SPoint m_P1;
		SPoint m_P2;
		SPoint m_P3;
	};
}
