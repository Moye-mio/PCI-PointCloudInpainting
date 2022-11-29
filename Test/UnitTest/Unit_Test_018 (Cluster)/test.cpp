#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/CrossPlane.ply");

class TestCluster : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}
};

TEST_F(TestCluster, DT_InValidInput)
{
	PC_t::Ptr pCloud;
	NormalPC_t::Ptr pNormals;
	GA::CCluster Cluster;
	ASSERT_DEATH(Cluster.setData(pCloud, pNormals), "");
}

TEST_F(TestCluster, NT_OneData)
{
	PC_t::Ptr pCloud(new PC_t);
	NormalPC_t::Ptr pNormals(new NormalPC_t);

	pCloud->emplace_back(Point_t(0, 0, 0));
	pNormals->emplace_back(Normal_t(0, 0, 1));

	GA::CCluster Cluster;
	Cluster.setData(pCloud, pNormals);
	Cluster.calcGD();
	ASSERT_EQ(Cluster.computePointFitness(pCloud->at(0), pNormals->at(0)), 1.0f);
}

TEST_F(TestCluster, NT_TwoData)
{
	PC_t::Ptr pCloud(new PC_t);
	NormalPC_t::Ptr pNormals(new NormalPC_t);

	pCloud->emplace_back(Point_t(0, 0, 0));
	pCloud->emplace_back(Point_t(3.2f, 0.8f, 1.1f));
	pNormals->emplace_back(Normal_t(0, 0, 1));
	pNormals->emplace_back(Normal_t(0, 0, 1));

	GA::CCluster Cluster;
	Cluster.setData(pCloud, pNormals);
	Cluster.calcGD();
	ASSERT_LT(Cluster.computePointFitness(pCloud->at(0), pNormals->at(0)) - 1.5f, 0.1f);
}

TEST_F(TestCluster, NT_ThreeData)
{
	PC_t::Ptr pCloud(new PC_t);
	NormalPC_t::Ptr pNormals(new NormalPC_t);

	pCloud->emplace_back(Point_t(0, 0, 0));
	pCloud->emplace_back(Point_t(1, 0, 0));
	pCloud->emplace_back(Point_t(0, 1, 0));
	pNormals->emplace_back(Normal_t(0, 0, 1));
	pNormals->emplace_back(Normal_t(0, 0, 1));
	pNormals->emplace_back(Normal_t(0, 0, 1));

	GA::CCluster Cluster;
	Cluster.setData(pCloud, pNormals);
	Cluster.calcGD();
	ASSERT_LT(Cluster.computePointFitness(pCloud->at(0), pNormals->at(0)) - 0.43f, 0.01f);
}