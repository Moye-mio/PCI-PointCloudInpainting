#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/CrossPlane.ply");
const std::string ModelPath2 = TESTMODEL_DIR + std::string("/GT/GT_CrossPlane.ply");

class TestVoxelization : public testing::Test
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

TEST_F(TestVoxelization, DT_InValidInput)
{
	core::CVoxelization Voxelization;
	ASSERT_DEATH(Voxelization.setCloud(PC_t::Ptr()), "");
	ASSERT_DEATH(Voxelization.generate(0), "");
	ASSERT_DEATH(Voxelization.generate(0.1f), "");
}

TEST_F(TestVoxelization, NT_OnePoint)
{
	PC_t::Ptr pCloud(new PC_t);
	pCloud->emplace_back(Point_t(0, 0, 0));
	pCloud->emplace_back(Point_t(1, 0, 0));
	pCloud->emplace_back(Point_t(2, 0, 0));
	pCloud->emplace_back(Point_t(3, 0, 0));

	std::vector<std::pair<Eigen::Vector3i, Point_t>> Voxels;
	core::CVoxelization Voxelization;
	Voxelization.setCloud(pCloud);
	Voxelization.generate(0.5f);
	Voxelization.dumpVoxel(Voxels);
	ASSERT_EQ(Voxels.size(), 4);
	for (int i = 0; i < pCloud->size(); i++)
		ASSERT_TRUE(isPointSame(pCloud->at(i), Voxels[i].second));
}

TEST_F(TestVoxelization, NT_CrossPlane)
{
	PC_t::Ptr pCloud = loadPC(ModelPath);

	float Dist = 0.4f;
	std::vector<std::pair<Eigen::Vector3i, Point_t>> Voxels;
	core::CVoxelization Voxelization;
	Voxelization.setCloud(pCloud);
	Voxelization.generate(Dist);
	Voxelization.dumpVoxel(Voxels);
	ASSERT_EQ(Voxels.size(), 554);

	PC_t::Ptr pData(new PC_t);
	for (const auto& e : Voxels)
		pData->emplace_back(e.second);
	pcl::io::savePLYFileBinary("Voxel.ply", *pData);

	std::cout << "Voxel Size: " << Voxels.size() << std::endl;
	for (const auto& e : Voxels)
		std::cout << "(" << e.first.x() << ", " << e.first.y() << ", " << e.first.z() << "), " << e.second << std::endl;
}

TEST_F(TestVoxelization, NT_GT_CrossPlane)
{
	PC_t::Ptr pCloud = loadPC(ModelPath2);

	float Dist = 1.0f;
	std::vector<std::pair<Eigen::Vector3i, Point_t>> Voxels;
	core::CVoxelization Voxelization;
	Voxelization.setCloud(pCloud);
	Voxelization.generate(Dist);
	Voxelization.dumpVoxel(Voxels);
	EXPECT_EQ(Voxels.size(), 240);

	PC_t::Ptr pData(new PC_t);
	for (const auto& e : Voxels)
		pData->emplace_back(e.second);
	pcl::io::savePLYFileBinary("Voxel2.ply", *pData);

	std::cout << "Voxel Size: " << Voxels.size() << std::endl;
	for (const auto& e : Voxels)
		std::cout << "(" << e.first.x() << ", " << e.first.y() << ", " << e.first.z() << "), " << e.second << std::endl;
}
