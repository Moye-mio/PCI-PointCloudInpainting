#include "pch.h"

bool isPointSame(const Point_t& vLhs, const Point_t& vRhs)
{
	if (vLhs.x == vRhs.x && vLhs.y == vRhs.y && vLhs.z == vRhs.z && vLhs.r == vRhs.r && vLhs.g == vRhs.g && vLhs.b == vRhs.b && vLhs.a == vRhs.a)
		return true;
	else
		return false;
}

TEST(TestControlPointGenerator, DT_EmptyData) {
	core::CControlPointGenerator Generator;
	EXPECT_FALSE(Generator.run(Eigen::Matrix<Point_t, -1, -1>()));
}

TEST(TestControlPointGenerator, NT_InclinePlane) {
	Eigen::Matrix<Point_t, -1, -1> Data, ControlPoints;
	Data.resize(2, 2);
	Data.coeffRef(0, 0) = Point_t(1, 1, 1);
	Data.coeffRef(0, 1) = Point_t(1, 2, 2);
	Data.coeffRef(1, 0) = Point_t(2, 1, 2);
	Data.coeffRef(1, 1) = Point_t(2, 2, 3);

	core::CControlPointGenerator Generator;
	EXPECT_TRUE(Generator.run(Data));
	Generator.dumpControlPoints(ControlPoints);
	for (int i = 0; i < ControlPoints.rows(); i++)
		for (int k = 0; k < ControlPoints.cols(); k++)
			EXPECT_TRUE(isPointSame(ControlPoints.coeff(i, k), Point_t(i, k, i + k - 1)));
}