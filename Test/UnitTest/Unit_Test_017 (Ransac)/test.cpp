#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/CrossPlane.ply");

class TestPlaneFitting : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}
};

TEST_F(TestPlaneFitting, DT_inValidThreshold)
{
	PC_t::Ptr pCloud;
	PC_t::Ptr pCloud2(new PC_t);
	PC_t::Ptr pCloud3(new PC_t); pCloud3->push_back(Point_t(0, 0, 0));
	core::CPlaneFitting Fitting;
	ASSERT_DEATH(Fitting.fitRansacPlane(pCloud, 0.1), "");
	ASSERT_DEATH(Fitting.fitRansacPlane(pCloud2, 0.1), "");
	ASSERT_DEATH(Fitting.fitRansacPlane(pCloud3, 0), "");
}

TEST_F(TestPlaneFitting, DT_OneData)
{
	PC_t::Ptr pCloud(new PC_t); 
	pCloud->push_back(Point_t(0, 0, 0));
	pCloud->push_back(Point_t(1, 0, 0));
	core::CPlaneFitting Fitting;
	ASSERT_DEATH(Fitting.fitRansacPlane(pCloud, 0.1), "");
}

TEST_F(TestPlaneFitting, NT_RegularPoints)
{
	PC_t::Ptr pCloud(new PC_t);
	pCloud->push_back(Point_t(0, 0, 0));
	pCloud->push_back(Point_t(1, 0, 0));
	pCloud->push_back(Point_t(0, 1, 0));
	pCloud->push_back(Point_t(1, 1, 0));
	core::CPlaneFitting Fitting;
	common::SPlane Plane = Fitting.fitRansacPlane(pCloud, 0.1);
	Eigen::Vector4f GTCoefs = { 0, 0, 1, 0 };

	for (int i = 0; i < 4; i++)
	{
		ASSERT_EQ(Plane[i], GTCoefs[i]);
	}
}

TEST_F(TestPlaneFitting, NT_RandomPoints)
{
	PC_t::Ptr pCloud(new PC_t);
	
	for (int k = 0; k < 3; k++)
		for (int i = 0; i < 1000; i++)
			pCloud->push_back(Point_t(hiveMath::hiveGenerateRandomReal(0.0f, 100.0f), hiveMath::hiveGenerateRandomReal(0.0f, 100.0f), k));

	core::CPlaneFitting Fitting;
	common::SPlane Plane = Fitting.fitRansacPlane(pCloud, 1.1f);
	Eigen::Vector4f GTCoefs = { 0, 0, 1, -1 };

	for (int i = 0; i < 4; i++)
	{
		ASSERT_EQ(Plane[i], GTCoefs[i]);
	}
}

TEST_F(TestPlaneFitting, NT_FailModel)
{
	PC_t::Ptr pCloud(new PC_t);
	pcl::io::loadPLYFile("fail.ply", *pCloud);
	_ASSERTE(pCloud->size());

	core::CPlaneFitting Fitting;
	common::SPlane Plane = Fitting.fitRansacPlane(pCloud, 0.3f);
	Eigen::Vector4f GTCoefs = { 1, 0, 0, 0 };

	for (int i = 0; i < 4; i++)
	{
		ASSERT_EQ(Plane[i], GTCoefs[i]);
	}
}