#include "pch.h"

class TestMultiLayerBSS : public testing::Test
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

TEST_F(TestMultiLayerBSS, DT_InValidLayers)
{
	core::CMultiLayerBSplineSurface Surface(3, 0);
	ASSERT_DEATH(Surface.setLayer(0), "");
}

TEST_F(TestMultiLayerBSS, DT_InValidDegree)
{
	ASSERT_DEATH(core::CMultiLayerBSplineSurface Surface(0, 0), "");
}

TEST_F(TestMultiLayerBSS, DT_InValidPoint)
{
	core::CMultiLayerBSplineSurface Surface(3, 0);
	Surface.setLayer(3);
	Eigen::Vector2f UV;
	ASSERT_DEATH(Surface.calcProj(core::SPoint(), UV), "");
}

TEST_F(TestMultiLayerBSS, NT_SubdivideSurface)
{
	Eigen::Matrix<core::SPoint, 4, 4> ControlPoints;
	for (int i = 0; i < 4; i++)
		for (int k = 0; k < 4; k++)
			ControlPoints.coeffRef(i, k) = core::SPoint(Eigen::Vector3f(i, k, 0));

	std::vector<float> Error;
	core::CMultiLayerBSplineSurface Surface(3, 0);
	Surface.setLayer(2);
	Surface.setMaxSub(8);
	Surface.setControlPoints(ControlPoints);
	Surface.setIsCalcError(true);
	Eigen::Vector2f UV;
	float Dist = Surface.calcProj(core::SPoint(Eigen::Vector3f(1.5f, 1.5f, 0.0f)), UV);
	std::cout << Dist << "\t" << UV << std::endl;
	Surface.dumpError(Error);
	for (auto& e : Error)
		std::cout << e << std::endl;
	ASSERT_LT(Dist, m_Epsilon);
}

