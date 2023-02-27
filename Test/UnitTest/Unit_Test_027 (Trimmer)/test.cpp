#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/CrossPlane.ply");

class TestTrimmer : public testing::Test
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

	void generateConcave(std::vector<std::pair<Eigen::Vector3i, Point_t>>& voData, std::vector<std::pair<Eigen::Vector3i, Point_t>>& voGTData)
	{
		int Size = 4;
		for (int i = 0; i < 4; i++)
			for (int k = 0; k < 4; k++)
				for (int m = 0; m < 4; m++)
				{
					if ((i == 1 || i == 2) && m > 0) continue;
					voGTData.emplace_back(std::make_pair(Eigen::Vector3i(i, k, m), Point_t(i, k, m)));

					if ((i == 1 || i == 2) && (k == 1 || k == 2)) continue;
					voData.emplace_back(std::make_pair(Eigen::Vector3i(i, k, m), Point_t(i, k, m)));
				}
	}

	bool isPointSame(const Point_t& vLhs, const Point_t& vRhs) const
	{
		if (vLhs.getVector3fMap() != vRhs.getVector3fMap()) return false;
		if (vLhs.getRGBVector3i() != vRhs.getRGBVector3i()) return false;
		return true;
	}

	void trimSortedData(const std::vector<std::pair<Eigen::Vector3i, Point_t>> vVoxels, Eigen::Matrix<Point_t, -1, -1>& voPoints, float vScale = 1.0f)
	{
		core::CTrimmer Trimmer;
		Trimmer.setData(vVoxels, vScale);
		Trimmer.fillAndTrim();
		Trimmer.dumpSorted(voPoints);
	}
};

TEST_F(TestTrimmer, DT_InValidInput)
{
	core::CTrimmer Trimmer;
	ASSERT_DEATH(Trimmer.fillAndTrim(), "");
}

TEST_F(TestTrimmer, NT_SimplePlane)
{
	std::vector<std::pair<Eigen::Vector3i, Point_t>> Voxels;
	Voxels.emplace_back(std::make_pair(Eigen::Vector3i(0, 0, 0), Point_t(0, 0, 0)));
	Voxels.emplace_back(std::make_pair(Eigen::Vector3i(1, 0, 0), Point_t(2, 0, 0)));
	Voxels.emplace_back(std::make_pair(Eigen::Vector3i(0, 1, 0), Point_t(0, 2, 0)));
	Voxels.emplace_back(std::make_pair(Eigen::Vector3i(1, 1, 0), Point_t(2, 2, 0)));

	Eigen::Matrix<Point_t, -1, -1> Points;
	trimSortedData(Voxels, Points);
	ASSERT_EQ(Points.rows(), 2);
	ASSERT_EQ(Points.cols(), 2);
	for (int i = 0; i < 2; i++)
		for (int k = 0; k < 2; k++)
			EXPECT_TRUE(isPointSame(Points.coeff(k, i), Voxels[i * 2 + k].second));
}

TEST_F(TestTrimmer, NT_OneEmptySimplePlane)
{
	int Size = 3;
	std::vector<std::pair<Eigen::Vector3i, Point_t>> Voxels, GTVoxels;
	for (int i = 0; i < Size; i++)
		for (int k = 0; k < Size; k++)
		{
			GTVoxels.emplace_back(std::make_pair(Eigen::Vector3i(i, k, 0), Point_t(i * 2, k * 2, 0)));
			if (i == 1 && k == 1) continue;
			Voxels.emplace_back(std::make_pair(Eigen::Vector3i(i, k, 0), Point_t(i * 2, k * 2, 0)));
		}

	Eigen::Matrix<Point_t, -1, -1> Points;
	trimSortedData(Voxels, Points);
	ASSERT_EQ(Points.rows(), Size);
	ASSERT_EQ(Points.cols(), Size);
	for (int i = 0; i < Size; i++)
		for (int k = 0; k < Size; k++)
		{
			{
				std::cout << i << ", " << k << std::endl;
				std::cout << "Trimmed: " << Points(i, k) << std::endl;
				std::cout << "GTVoxel: " << GTVoxels[i * Size + k].second << std::endl;
			}

			EXPECT_TRUE(isPointSame(Points.coeff(i, k), GTVoxels[i * Size + k].second));
		}
}

TEST_F(TestTrimmer, NT_ConcaveWithEmpty)
{
	Eigen::Matrix<Point_t, -1, -1> Points, GTPoints;
	std::vector<std::pair<Eigen::Vector3i, Point_t>> Voxels, GTVoxels;
	generateConcave(Voxels, GTVoxels);
	trimSortedData(Voxels, Points);
	trimSortedData(GTVoxels, GTPoints);

	for (int i = 0; i < Points.rows(); i++)
		for (int k = 0; k < Points.cols(); k++)
			EXPECT_TRUE(isPointSame(Points.coeff(i, k), GTPoints.coeff(i, k)));
}

TEST_F(TestTrimmer, NT_NoEmptyCrossPlane)
{
	PC_t::Ptr pCloud = loadPC(ModelPath);
	float Dist = 0.4f;
	std::vector<std::pair<Eigen::Vector3i, Point_t>> Voxels;
	core::CVoxelization Voxelization;
	Voxelization.setCloud(pCloud);
	Voxelization.generate(Dist);
	Voxelization.dumpVoxel(Voxels);
	ASSERT_EQ(Voxels.size(), 554);

	Eigen::Matrix<Point_t, -1, -1> Points;
	trimSortedData(Voxels, Points, Dist);
	PC_t::Ptr pData(new PC_t);
	for (int i = 0; i < Points.rows(); i++)
		for (int k = 0; k < Points.cols(); k++)
			pData->emplace_back(Points.coeff(i, k));

	pcl::io::savePLYFileBinary("Voxel.ply", *pData);
}