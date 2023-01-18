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

	void generateLPoints(Eigen::Matrix<core::SPoint, -1, -1>& voVertices, int vRows /* x */, int vCols /* y */, int vDepths /* z */)
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
};

TEST_F(TestMultiLayerSurface, DT_InValidInput)
{
	core::CMultilayerSurface Surface(3);
	EXPECT_FALSE(Surface.setSubLayer(0));
	EXPECT_FALSE(Surface.setSubNumber(0));
	auto r = Surface.calcProj(core::SPoint());
	EXPECT_FALSE(r.has_value());
}

TEST_F(TestMultiLayerSurface, NT_Plane)
{
	Eigen::Matrix<core::SPoint, -1, -1> Points;
	EXPECT_TRUE(generatePlanePoints(Points));

	core::CMultilayerSurface Surface(3);
	Surface.setControlPoints(Points);
	auto r = Surface.calcProj(Points.coeff(0, 0));
	EXPECT_TRUE(r.has_value());
}

TEST_F(TestMultiLayerSurface, NT_LSurface)
{
	int Rows = 5;
	int Cols = 5;
	int Depths = 5;
	int Offset = 6;
	int Degree = 3;
	int Scale = 9;

	Eigen::Matrix<core::SPoint, -1, -1> ControlPoints;
	generateLPoints(ControlPoints, Rows, Cols, Depths);

	std::vector<float> Error;
	core::CMultilayerSurface Surface(3, 0);
	Surface.setIsSaveMesh(true);
	Surface.setControlPoints(ControlPoints);
	Surface.setIsCalcError(true);

	std::vector<float> Dists;
	for (int i = 0; i < ControlPoints.rows(); i++)
		for (int k = 0; k < ControlPoints.cols(); k++)
		{
			std::cout << "Point: (" << i << ", " << k << ")" << std::endl;
			auto r = Surface.calcProj(ControlPoints(i, k));
			if (r.has_value())
			{
				float Dist = r.value()._Dist;
				Dists.push_back(Dist);
			}
			else
				Dists.push_back(-FLT_MAX);
		}

	std::cout << "Dist: " << std::endl;
	int N = 0;
	for (int i = 0; i < ControlPoints.rows(); i++)
		for (int k = 0; k < ControlPoints.cols(); k++)
		{
			if (Dists[N] == -FLT_MAX)
				std::cout << "Point: (" << i << ", " << k << ")\tDist: NULL" << std::endl;
			else
				std::cout << "Point: (" << i << ", " << k << ")\tDist: " << Dists[N] << std::endl;
			N++;
		}

}