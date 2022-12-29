#pragma once
#include <Eigen/Eigen>

namespace core
{
	using SPoint = Eigen::Vector3f;

	struct STriangle
	{
		STriangle() = default;
		STriangle(const SPoint& vPoint1, const SPoint& vPoint2, const SPoint& vPoint3)
			: _Point1(vPoint1)
			, _Point2(vPoint2)
			, _Point3(vPoint3) {}

		SPoint _Point1;
		SPoint _Point2;
		SPoint _Point3;

		SPoint& operator[](unsigned int i) {
			_ASSERTE(i < size());
			switch (i)
			{
			case 0:
				return _Point1;
			case 1:
				return _Point2; 
			case 2:
				return _Point3;
			}
		}

		int size()
		{
			return 3;
		}

	};
}
