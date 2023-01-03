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
	core::SPoint Point4(Eigen::Vector3f(0.5, 1, 0));
	Eigen::Vector3f RayDir(0, 0, 1);
	core::CTriangle Tri(core::SPoint(Eigen::Vector3f(0, 0, 0)), core::SPoint(Eigen::Vector3f(2, 2, 0)), core::SPoint(Eigen::Vector3f(0, 1, 0)));
	ASSERT_EQ(Tri.isRayIntersection(Point, RayDir), true);
	ASSERT_EQ(Tri.isRayIntersection(Point2, RayDir), false);
	ASSERT_EQ(Tri.isRayIntersection(Point3, RayDir), true);
	ASSERT_EQ(Tri.isRayIntersection(Point3, -RayDir), true);
	ASSERT_EQ(Tri.isRayIntersection(Point4, -RayDir), true);
}

TEST_F(TestTriangle, NT_CalcBaryCoor)
{
	core::SPoint Point(Eigen::Vector3f(0, 0, 0));
	core::SPoint Point2(Eigen::Vector3f(1, 0, 0));
	core::SPoint Point3(Eigen::Vector3f(0, 1, 0));
	core::SPoint Point4(Eigen::Vector3f(0.5, 0.5, 0));
	core::CTriangle Tri(core::SPoint(Eigen::Vector3f(0, 0, 0)), core::SPoint(Eigen::Vector3f(1, 0, 0)), core::SPoint(Eigen::Vector3f(0, 1, 0)));
	Eigen::Vector3f Coor, Coor2, Coor3, Coor4;
	Tri.calcBaryCoor(Point, Coor);
	Tri.calcBaryCoor(Point2, Coor2);
	Tri.calcBaryCoor(Point3, Coor3);
	Tri.calcBaryCoor(Point4, Coor4);
	ASSERT_EQ(Coor, Eigen::Vector3f(1, 0, 0));
	ASSERT_EQ(Coor2, Eigen::Vector3f(0, 1, 0));
	ASSERT_EQ(Coor3, Eigen::Vector3f(0, 0, 1));
	ASSERT_EQ(Coor4, Eigen::Vector3f(0, 0.5, 0.5));

	for (int i = 0; i < 10; i++)
	{
		auto RealSet = hiveMath::hiveGenerateRandomRealSet(0.0f, 0.5f, 2);
		float W1 = RealSet[0];
		float W2 = RealSet[1];
		float W3 = 1.0f - W1 - W2;
		Eigen::Vector3f Point5 = Tri[0] * W1 + Tri[1] * W2 + Tri[2] * W3;

		Eigen::Vector3f Coor5;
		Tri.calcBaryCoor(core::SPoint(Point5), Coor5);

		std::cout << "W: " << W1 << ", " << W2 << ", " << W3 << std::endl;
		std::cout << "C: " << Coor5[0] << ", " << Coor5[1] << ", " << Coor5[2] << std::endl;

		ASSERT_LT((Coor5 - Eigen::Vector3f(W1, W2, W3)).norm(), m_Epsilon);
	}
}