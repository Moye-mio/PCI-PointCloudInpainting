#include "pch.h"

class TestTriangle : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}

	float m_Epsilon = 0.00001f;
};

TEST_F(TestTriangle, DT_NoPoint)
{
	core::CTriangle Tri;
	common::SPlane Plane;
	ASSERT_DEATH(Tri.calcPlane(Plane), "");
}

TEST_F(TestTriangle, DT_SamePoint)
{
	core::CTriangle Tri(core::SPoint(Eigen::Vector3f(0, 0, 0)), core::SPoint(Eigen::Vector3f(1, 0, 0)), core::SPoint(Eigen::Vector3f(0, 0, 0)));
	common::SPlane Plane;
	ASSERT_DEATH(Tri.calcPlane(Plane), "");
}

TEST_F(TestTriangle, DT_ThreePointSameLine)
{
	core::CTriangle Tri(core::SPoint(Eigen::Vector3f(1, 0, 0)), core::SPoint(Eigen::Vector3f(2, 0, 0)), core::SPoint(Eigen::Vector3f(3, 0, 0)));
	common::SPlane Plane;
	ASSERT_DEATH(Tri.calcPlane(Plane), "");
}

TEST_F(TestTriangle, NT_SimpleTriangle)
{
	core::CTriangle Tri(core::SPoint(Eigen::Vector3f(1, 0, 0)), core::SPoint(Eigen::Vector3f(2, 0, 0)), core::SPoint(Eigen::Vector3f(0, 1, 0)));
	common::SPlane Plane, PlaneGT(0, 0, 1, 0);
	Tri.calcPlane(Plane);
	for (int i = 0; i < 4; i++)
		ASSERT_LT(Plane[i] - PlaneGT[i], m_Epsilon);
}

TEST_F(TestTriangle, NT_ObliqueTriangle)
{
	core::CTriangle Tri(core::SPoint(Eigen::Vector3f(0, 0, 0)), core::SPoint(Eigen::Vector3f(2, 2, 2)), core::SPoint(Eigen::Vector3f(0, 1, 0)));
	common::SPlane Plane, PlaneGT(1, 0, -1, 0);
	Tri.calcPlane(Plane);
	for (int i = 0; i < 4; i++)
		ASSERT_LT(Plane[i] - PlaneGT[i], m_Epsilon);
}

TEST_F(TestTriangle, NT_RayIntersection)
{
	core::SPoint Point(Eigen::Vector3f(0, 0, 1));
	core::SPoint Point2(Eigen::Vector3f(-1, -1, 2));
	core::SPoint Point3(Eigen::Vector3f(0.5, 1, -1));
	Eigen::Vector3f RayDir(0, 0, 1);
	core::CTriangle Tri(core::SPoint(Eigen::Vector3f(0, 0, 0)), core::SPoint(Eigen::Vector3f(2, 2, 0)), core::SPoint(Eigen::Vector3f(0, 1, 0)));
	ASSERT_EQ(Tri.isRayIntersection(Point, RayDir), true);
	ASSERT_EQ(Tri.isRayIntersection(Point2, RayDir), false);
	ASSERT_EQ(Tri.isRayIntersection(Point3, RayDir), true);
	ASSERT_EQ(Tri.isRayIntersection(Point3, -RayDir), true);
}

