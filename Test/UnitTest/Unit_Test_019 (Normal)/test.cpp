#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/UniformRaw.ply");
const std::string ModelPath2 = TESTMODEL_DIR + std::string("/Trimmed/Surface/Surface.ply");

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
		float D1 = (vLhs - vRhs).norm();
		Eigen::Vector3f Inv = vRhs * -1;
		float D2 = (vLhs - Inv).norm();
		return std::min(D1, D2);
	}

	PC_t::Ptr loadCloud(const std::string& vModelPath)
	{
		std::string FileName = hiveUtility::hiveLocateFile(vModelPath);
		_HIVE_EARLY_RETURN(FileName.empty(), "Load Cloud: File is empty", nullptr);

		PC_t::Ptr pCloud(new PC_t);
		_HIVE_EARLY_RETURN(pcl::io::loadPLYFile<Point_t>(ModelPath, *pCloud) == -1, "Load Cloud: Read Cloud failed", nullptr);
		_HIVE_EARLY_RETURN(pCloud->size() == 0, "Load Cloud: Cloud is empty", nullptr);

		return pCloud;
	}

	float m_Epsilon = 0.00001;
};

TEST_F(TestNormal, DT_InValidInput)
{
	core::CNormalEstimator Estimator;
	PC_t::Ptr pCloud;
	PC_t::Ptr pCloud2(new PC_t);
	EXPECT_FALSE(Estimator.setCloud(pCloud));
	EXPECT_FALSE(Estimator.setCloud(pCloud2));
	pCloud2->push_back(Point_t(0, 0, 0));
	Estimator.setCloud(pCloud2);
	EXPECT_FALSE(Estimator.compute(0.0f));
	EXPECT_FALSE(Estimator.compute(-1));
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
	Estimator.compute(2.0f);
	Estimator.dumpNormals(pNormals);

	for (int i = 0; i < 100; i++)
	{
		ASSERT_EQ(pNormals->at(i).normal_x, 0);
		ASSERT_EQ(pNormals->at(i).normal_y, 0);
		ASSERT_EQ(pNormals->at(i).normal_z, 1);
	}
}

TEST_F(TestNormal, NT_RightAngle)
{
	PC_t::Ptr pCloud = loadCloud(ModelPath);

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

TEST_F(TestNormal, NT_Bending)
{
	PC_t::Ptr pCloud = loadCloud(ModelPath2);

	NormalPC_t::Ptr pNormals;
	core::CNormalEstimator Estimator;
	Estimator.setCloud(pCloud);
	Estimator.compute(4);
	Estimator.dumpNormals(pNormals);

	EXPECT_TRUE(pNormals->size() == pCloud->size());
	for (int i = 0; i < pCloud->size(); i++)
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR6("(%1%, %2%, %3%)-(%4%, %5%, %6%)", pCloud->at(i).x, pCloud->at(i).y, pCloud->at(i).z, pNormals->at(i).normal_x, pNormals->at(i).normal_y, pNormals->at(i).normal_z));
}