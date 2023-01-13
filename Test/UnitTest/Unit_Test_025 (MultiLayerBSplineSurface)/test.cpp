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

	void generatePoints(Eigen::Matrix<core::SPoint, -1, -1>& voVertices, int vRows /* x */, int vCols /* y */, int vDepths /* z */)
	{
		voVertices.resize(vRows + vCols - 1, vDepths);

		int Count = -1;
		for (int i = 0; i < vRows; i++)
			for (int k = vCols - 1; k >= 0; k--)
				for (int m = 0; m < vDepths; m++)
					if (i == 0 || k == 0)
					{
						Count++;
						core::SPoint Point;
						Point.x() = i * 0.2 - 0.5f;
						Point.y() = k * 0.2 - 0.5f;
						Point.z() = m * 0.2;
						voVertices.coeffRef(Count / vDepths, m) = Point;
					}

		std::cout << "Control Points: " << voVertices.size() << std::endl;
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
	auto r = Surface.calcProj(core::SPoint(Eigen::Vector3f(1.5f, 1.5f, 0.0f)), UV);
	float Dist = r.value();
	std::cout << Dist << "\t" << UV << std::endl;
	Surface.dumpError(Error);
	for (auto& e : Error)
		std::cout << e << std::endl;
	ASSERT_LT(Dist, m_Epsilon);

	r = Surface.calcProj(core::SPoint(Eigen::Vector3f(1.5f, 1.5f, 1.0f)), UV);
	Dist = r.value();
	std::cout << Dist << "\t" << UV << std::endl;
}

//TEST_F(TestMultiLayerBSS, NT_LSurface)
//{
//	int Rows = 5;
//	int Cols = 5;
//	int Depths = 5;
//	int Offset = 6;
//	int Degree = 3;
//	int Scale = 9;
//
//	Eigen::Matrix<core::SPoint, -1, -1> ControlPoints;
//	generatePoints(ControlPoints, Rows, Cols, Depths);
//
//	std::vector<float> Error;
//	core::CMultiLayerBSplineSurface Surface(3, 0);
//	Surface.setLayer(2);
//	Surface.setMaxSub(8);
//	Surface.setControlPoints(ControlPoints);
//	Surface.setIsCalcError(true);
//	Eigen::Vector2f UV;
//
//	for (int i = 0; i < ControlPoints.rows(); i++)
//		for (int k = 0; k < ControlPoints.cols(); k++)
//		{
//			auto r = Surface.calcProj(ControlPoints(i, k), UV);
//			if (r.has_value())
//			{
//				float Dist = r.value();
//				std::cout << Dist << "\t" << UV << std::endl;
//			}
//			else
//				std::cout << "NULL" << std::endl;
//		}
//}