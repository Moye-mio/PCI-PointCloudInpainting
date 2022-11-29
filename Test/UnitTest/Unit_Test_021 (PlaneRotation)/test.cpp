#include "pch.h"

class TestPlaneRotation : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}
};

TEST_F(TestPlaneRotation, NT_SpecialPlane)
{
	Eigen::Vector4f Plane(1, -1, 0, 1);
	float Divisor = std::sqrtf(Plane[0] * Plane[0] + Plane[1] * Plane[1] + Plane[2] * Plane[2]);

	Eigen::Matrix3f Rotation;
	float Angle = 90 * std::numbers::pi / 180;
	Eigen::Vector3f Axis(-Plane[1] / Divisor, Plane[0] / Divisor, 0);
	float K = 1 - std::cos(Angle);

	Rotation << Axis[0] * Axis[0] * K + std::cos(Angle), Axis[0] * Axis[1] * K, Axis[1] * std::sin(Angle),
		Axis[0] * Axis[1] * K, Axis[1] * Axis[1] * K + std::cos(Angle), -Axis[0] * std::sin(Angle),
		-Axis[1] * std::sin(Angle), Axis[0] * std::sin(Angle), std::cos(Angle);

	std::vector<Eigen::Vector3f> Points;
	Points.emplace_back(Eigen::Vector3f(-1, 0, 0)); 
	Points.emplace_back(Eigen::Vector3f(0, 1, 0));
	Points.emplace_back(Eigen::Vector3f(1, 2, 0));
	Points.emplace_back(Eigen::Vector3f(1, 2, 1));
	Points.emplace_back(Eigen::Vector3f(1, 2, 2));
	Points.emplace_back(Eigen::Vector3f(1, 2, 3));
	Points.emplace_back(Eigen::Vector3f(1, 2, 4));

	for (auto& e : Points)
		_ASSERTE(Plane[0] * e[0] + Plane[1] * e[1] + Plane[2] * e[2] + Plane[3] == 0);

	std::cout << Rotation << std::endl << std::endl;

	float z = -FLT_MAX;
	for (auto& e : Points)
	{
		Eigen::Vector3f p = Rotation * e;
		std::cout << "(" << p[0] << ", " << p[1] << ", " << p[2] << ")\n";
		if (z == -FLT_MAX)
			z = p[2];
		else
			ASSERT_LT(z - p[2], 0.000001);
	}

	Eigen::Vector3f p = Rotation * Eigen::Vector3f(Plane[0], Plane[1], Plane[2]);
	std::cout << "\n" << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
}
