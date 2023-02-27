#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/CrossPlane.ply");
const std::string ModelPath2 = TESTMODEL_DIR + std::string("/GT/GT_CrossPlane.ply");

class TestBiasedVoxelization : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}

	PC_t::Ptr loadPC(const std::string& vPath)
	{
		std::string FileName = hiveUtility::hiveLocateFile(vPath);
		_ASSERTE(!FileName.empty());

		PC_t::Ptr pCloud(new PC_t);
		int r = pcl::io::loadPLYFile<Point_t>(FileName, *pCloud);
		_ASSERTE(r != -1);
		_ASSERTE(pCloud->size());
		std::cout << "Model Point Size: " << pCloud->size() << std::endl;
		return pCloud;
	}

	bool isPointSame(const Point_t& vLhs, const Point_t& vRhs) const
	{
		if (vLhs.getVector3fMap() != vRhs.getVector3fMap()) return false;
		if (vLhs.getRGBVector3i() != vRhs.getRGBVector3i()) return false;
		return true;
	}
};

TEST_F(TestBiasedVoxelization, DT_InValidInput)
{
	core::CBiasedVoxelization Voxelization;
	EXPECT_FALSE(Voxelization.setCloud(PC_t::Ptr()), "");
	EXPECT_FALSE(Voxelization.generate(0), "");
	EXPECT_FALSE(Voxelization.generate(0.1f), "");
}

TEST_F(TestBiasedVoxelization, NT_OnePoint)
{
	PC_t::Ptr pCloud(new PC_t);
	pCloud->emplace_back(Point_t(0, 0, 0));
	pCloud->emplace_back(Point_t(1, 0, 0));
	pCloud->emplace_back(Point_t(2, 0, 0));
	pCloud->emplace_back(Point_t(3, 0, 0));

	std::vector<std::pair<Eigen::Vector3i, Point_t>> Voxels;
	core::CBiasedVoxelization Voxelization;
	Voxelization.setCloud(pCloud);
	Voxelization.generate(0.5f);
	Voxelization.dumpVoxel(Voxels);
	EXPECT_EQ(Voxels.size(), 4);
	for (int i = 0; i < pCloud->size(); i++)
		EXPECT_TRUE(isPointSame(pCloud->at(i), Voxels[i].second));
}

TEST_F(TestBiasedVoxelization, NT_CrossPlane)
{
	PC_t::Ptr pCloud = loadPC(ModelPath);

	float Dist = 0.4f;
	std::vector<std::pair<Eigen::Vector3i, Point_t>> Voxels;
	core::CBiasedVoxelization Voxelization;
	Voxelization.setCloud(pCloud);
	Voxelization.generate(Dist);
	Voxelization.dumpVoxel(Voxels);
	EXPECT_EQ(Voxels.size(), 903);

	PC_t::Ptr pData(new PC_t);
	for (const auto& e : Voxels)
		pData->emplace_back(e.second);
	pcl::io::savePLYFileBinary("Voxel.ply", *pData);
}

TEST_F(TestBiasedVoxelization, NT_GT_CrossPlane)
{
	PC_t::Ptr pCloud = loadPC(ModelPath2);

	float Dist = 1.0f;
	std::vector<std::pair<Eigen::Vector3i, Point_t>> Voxels;
	core::CBiasedVoxelization Voxelization;
	Voxelization.setCloud(pCloud);
	//Voxelization.setDenoiseThres(2);
	Voxelization.generate(Dist);
	Voxelization.dumpVoxel(Voxels);
	EXPECT_EQ(Voxels.size(), 198);

	PC_t::Ptr pData(new PC_t);
	for (const auto& e : Voxels)
		pData->emplace_back(e.second);
	pcl::io::savePLYFileBinary("Voxel2.ply", *pData);

}
