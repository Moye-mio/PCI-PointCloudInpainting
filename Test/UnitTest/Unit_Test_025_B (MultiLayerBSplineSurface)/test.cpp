#include "pch.h"

#include "MultilayerSurface.h"

class TestMultiLayerSurface : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}

	bool generatePlanePoints(Eigen::Matrix<core::SPoint, -1, -1>& voPoints)
	{
		voPoints.resize(4, 4);
		for (int i = 0; i < 4; i++)
			for (int k = 0; k < 4; k++)
				voPoints.coeffRef(i, k) = core::SPoint(i, k, 0);
		return true;
	}
};

TEST_F(TestMultiLayerSurface, DT_InValidInput)
{
	core::CMultilayerSurface Surface(3);
	EXPECT_FALSE(Surface.setSubLayer(0));
	EXPECT_FALSE(Surface.setSubNumber(0));
	auto r = Surface.calcProj(core::SPoint());
	EXPECT_FALSE(r.has_value());
}

TEST_F(TestMultiLayerSurface, NT_)
{
	Eigen::Matrix<core::SPoint, -1, -1> Points;
	EXPECT_TRUE(generatePlanePoints(Points));

	core::CMultilayerSurface Surface(3);
	Surface.setControlPoints(Points);
	auto r = Surface.calcProj(Points.coeff(0, 0));
	EXPECT_TRUE(r.has_value());
}