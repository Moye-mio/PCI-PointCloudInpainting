#include "pch.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/UniformSeg.ply");

class TestHoleBorderExtracting : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}
};

TEST_F(TestHoleBorderExtracting, NT_Integration)
{
	float Radius = 0.5f;

	std::string FileName = hiveUtility::hiveLocateFile(ModelPath);
	ASSERT_FALSE(FileName.empty());

	PC_t::Ptr pData(new PC_t);
	ASSERT_TRUE(pcl::io::loadPLYFile<Point_t>(ModelPath, *pData) != -1);
	ASSERT_TRUE(pData->size());

	std::vector<int> BorderIndices;
	core::CBorderExtractor Extractor(pData);
	Extractor.Compute(Radius);
	Extractor.dumpBorderIndices(BorderIndices);

	PC_t::Ptr pBorder(new PC_t);
	for (auto e : BorderIndices)
		pBorder->push_back(pData->at(e));

	std::vector<std::vector<int>> Clusters;
	core::CRegionGrowing3D Growing(pBorder);
	Growing.Grow(Radius);
	Growing.dumpGrowingResult(Clusters);
	ASSERT_TRUE(Clusters.size());

	std::vector<int> ExtremePoints;
	core::CExtremePoint Extreme(pBorder);
	ASSERT_TRUE(Extreme.computeExtreme());
	Extreme.dumpExtremeIndices(ExtremePoints);

	for (int i = 0; i < ExtremePoints.size(); i++)
		std::cout << "Extreme " << i << ": " << pData->at(ExtremePoints[i]) << std::endl;

	std::cout << "0: " << pData->at(0) << std::endl;

	std::vector<bool> IsInCluster(Clusters.size(), false);
	for (auto p : ExtremePoints)
	{
		bool IsFind = false;
		int ClusterIndex = 0;
		for (auto& e : Clusters)
		{
			auto Iter = std::find(e.begin(), e.end(), p);
			if (Iter != e.end())
			{
				IsInCluster[ClusterIndex] = true;
				break;
			}
			ClusterIndex++;
		}
	}
	
	int i = 0;
	for (auto e : IsInCluster)
	{
		std::cout << "Cluster " << i << ": " << e << std::endl;
		if (e == true)	// Model border
		{
			for (auto p : Clusters[i])
				pBorder->at(p).r = 255;
		}
		else
		{
			for (auto p : Clusters[i])
				pBorder->at(p).g = 255;
		}
		i++;
	}

	pcl::io::savePLYFileBinary("Output.ply", *pBorder);
}

