#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/UniformRaw.ply");

class TestNormal : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}

	float calcVecDists(const Eigen::Vector3f& vLhs, const Eigen::Vector3f& vRhs)
	{
		float a = std::sqrtf((vLhs[0] - vRhs[0]) * (vLhs[0] - vRhs[0]) + (vLhs[1] - vRhs[1]) * (vLhs[1] - vRhs[1]) + (vLhs[2] - vRhs[2]) * (vLhs[2] - vRhs[2]));
		Eigen::Vector3f Inv = vRhs * -1;
		float b = std::sqrtf((vLhs[0] - Inv[0]) * (vLhs[0] - Inv[0]) + (vLhs[1] - Inv[1]) * (vLhs[1] - Inv[1]) + (vLhs[2] - Inv[2]) * (vLhs[2] - Inv[2]));

		return std::min(a, b);
	}

	float m_Epsilon = 0.00001;
};

TEST_F(TestNormal, DT_InValidInput)
{
	core::CNormalEstimator Estimator;
	PC_t::Ptr pCloud;
	PC_t::Ptr pCloud2(new PC_t);
	ASSERT_DEATH(Estimator.setCloud(pCloud), "");
	ASSERT_DEATH(Estimator.setCloud(pCloud2), "");
	pCloud2->push_back(Point_t(0, 0, 0));
	Estimator.setCloud(pCloud2);
	ASSERT_DEATH(Estimator.compute(0.0f), "");
}

TEST_F(TestNormal, NT_Plane)
{
	PC_t::Ptr pCloud(new PC_t);
	for (int i = 0; i < 10; i++)
		for (int k = 0; k < 10; k++)
			pCloud->push_back(Point_t(i, k, 0));

	NormalPC_t::Ptr pNormals;
	core::CNormalEstimator Estimator;
	Estimator.setCloud(pCloud);
	Estimator.compute(2);
	Estimator.dumpNormals(pNormals);

	for (int i = 0; i < 100; i++)
	{
		ASSERT_EQ(pNormals->at(i).normal_x, 0);
		ASSERT_EQ(pNormals->at(i).normal_y, 0);
		ASSERT_EQ(pNormals->at(i).normal_z, 1);
	}
}

TEST_F(TestNormal, NT_Bending)
{
	std::string FileName = hiveUtility::hiveLocateFile(ModelPath);
	ASSERT_FALSE(FileName.empty());

	PC_t::Ptr pCloud(new PC_t);
	ASSERT_TRUE(pcl::io::loadPLYFile<Point_t>(ModelPath, *pCloud) != -1);
	ASSERT_TRUE(pCloud->size());

	NormalPC_t::Ptr pNormals;
	core::CNormalEstimator Estimator;
	Estimator.setCloud(pCloud);
	Estimator.compute(1.0f);
	Estimator.dumpNormals(pNormals);

	int Count = 0;
	for (int i = 0; i < pCloud->size(); i++)
	{
		auto& e = pNormals->at(i);
		Eigen::Vector3f Normal{ e.normal_x, e.normal_y, e.normal_z };
		if (calcVecDists({ 1, 0, 0 }, Normal) < m_Epsilon || calcVecDists({ 0, 0, 1 }, Normal) < m_Epsilon)
			Count++;
	}

	std::cout << "Fit Standard Number: " << Count << ", Total Number: " << pCloud->size() << std::endl;
	ASSERT_TRUE(Count > pCloud->size() * 0.6);
}