#include "pch.h"

class TestSurface2CloudMapper : public testing::Test
{
protected:
	void SetUp() override
	{

	}

	void TearDown() override
	{
	}

	bool isPointSame(const Point_t& a, const Point_t b)
	{
		float Epsilon = 0.0001f;
		if (std::fabsf(a.x - b.x) < Epsilon && std::fabsf(a.y - b.y) < Epsilon && std::fabsf(a.z - b.z) < Epsilon && std::fabsf(a.r - b.r) < Epsilon && std::fabsf(a.g - b.g) < Epsilon && std::fabsf(a.b - b.b) < Epsilon && std::fabsf(a.a - b.a) < Epsilon)
			return true;
		else
			return false;
	}
};

TEST_F(TestSurface2CloudMapper, DT_ZeroNormal)
{
	core::CSurface2PCMapper Mapper;
	EXPECT_FALSE(Mapper.generatePoint(core::SVertex(), Eigen::Vector3f(), 1.0f));
	EXPECT_FALSE(Mapper.generatePoint(core::SVertex(), Eigen::Vector3f(1, 2, 3), 1.0f));
}

TEST_F(TestSurface2CloudMapper, NT_SomePoints)
{
	core::CSurface2PCMapper Mapper;
	EXPECT_TRUE(isPointSame(Mapper.generatePoint(core::SVertex(0, 0, 0), Eigen::Vector3f(0, 0, 1), 1).value(), Point_t(0, 0, 1)));
	EXPECT_TRUE(isPointSame(Mapper.generatePoint(core::SVertex(5, 4, 3), Eigen::Vector3f(1, 0, 0), 10).value(), Point_t(15, 4, 3)));
	EXPECT_TRUE(isPointSame(Mapper.generatePoint(core::SVertex(0, 0, 0), Eigen::Vector3f(std::sqrtf(0.5), std::sqrtf(0.5), 0), 10).value(), Point_t(7.071, 7.071, 0)));
}
