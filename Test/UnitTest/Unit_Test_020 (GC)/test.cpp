#include "pch.h"
#include <windows.h>

const std::string ModelPath = TESTMODEL_DIR + std::string("/UniformSeg.ply");
const std::string ModelPath2 = TESTMODEL_DIR + std::string("/Road_Seg_Sim.ply");
const std::string ModelPath3 = TESTMODEL_DIR + std::string("/Road-test.ply");

class TestGC : public testing::Test
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
		return pCloud;
	}

	float generateClusterFesture(const std::vector<int>& vSolution, std::vector<GA::CCluster>& voClusters, int vClusterSize, const PC_t::Ptr& vCloud, const NormalPC_t::Ptr& vNormals)
	{
		voClusters.clear();
		voClusters.shrink_to_fit();

		for (int i = 0; i < vClusterSize; i++)
		{
			PC_t::Ptr pNewCloud(new PC_t);
			NormalPC_t::Ptr pNewNormals(new NormalPC_t);

			for (int k = 0; k < vCloud->size(); k++)
				if (vSolution[k] == i)
				{
					pNewCloud->emplace_back(vCloud->at(k));
					pNewNormals->emplace_back(vNormals->at(k));
				}

			if (pNewCloud->size() == 0) continue;

			_ASSERTE(pNewCloud->size() == pNewNormals->size());

			GA::CCluster Cluster;
			Cluster.setData(pNewCloud, pNewNormals);
			bool r = Cluster.calcGD();
			_ASSERTE(r);

			voClusters.emplace_back(Cluster);
		}
		return true;
	}

	float calcMutatePR(const std::vector<float>& vFitness, int vCate)
	{
		return (1 - vFitness[vCate] / std::accumulate(vFitness.begin(), vFitness.end(), 0.0f));
	}
};

//TEST_F(TestGC, NT_Special)
//{
//	PC_t::Ptr pCloud = loadPC(ModelPath);
//	NormalPC_t::Ptr pNormals(new NormalPC_t);
//	core::CNormalEstimator Estimator;
//	Estimator.setCloud(pCloud);
//	Estimator.compute(0.3f);
//	Estimator.dumpNormals(pNormals);
//
//	std::vector<int> Solution;
//	for (auto& e : *pCloud)
//	{
//		if (e.r == 255)
//			Solution.push_back(0);
//		if (e.g == 255)
//			Solution.push_back(1);
//		if (e.b == 255)
//			Solution.push_back(2);
//	}
//
//	std::vector<GA::CCluster> Clusters;
//	generateClusterFesture(Solution, Clusters, 3, pCloud, pNormals);
//	std::vector<int> MutationSolution = Solution;
//
//	int n = 0;
//	for (int k = 0; k < pCloud->size(); k++)
//	{
//		auto& Point = pCloud->at(k);
//		auto& Normal = pNormals->at(k);
//		std::vector<float> PointFitness;
//		for (auto& e : Clusters)
//			PointFitness.push_back(e.computePointFitness(Point, Normal));
//
//		if (std::distance(PointFitness.begin(), std::max_element(PointFitness.begin(), PointFitness.end())) != Solution[k])
//		{
//			std::cout << PointFitness[0] << ", " << PointFitness[1] << ", " << PointFitness[2] << ", " << Solution[k] << std::endl;
//			n++;
//			if (pCloud->at(k).r == 255)
//				pCloud->at(k).g = 255;
//			else if (pCloud->at(k).g == 255)
//				pCloud->at(k).b = 255;
//			else if (pCloud->at(k).b == 255)
//				pCloud->at(k).r = 255;
//		}
//
//		float PR = calcMutatePR(PointFitness, Solution[k]);
//		if (hiveMath::hiveGenerateRandomReal(0.0f, 1.0f) < PR)
//			MutationSolution[k] = std::distance(PointFitness.begin(), std::max_element(PointFitness.begin(), PointFitness.end()));
//	}
//	std::cout << n << std::endl;
//	pcl::io::savePLYFileBinary("Output2.ply", *pCloud);
//}

TEST_F(TestGC, NT_Concave)
{
	SYSTEM_INFO SysInfo;
	GetSystemInfo(&SysInfo);
	printf("now system cpu num is %d\n", SysInfo.dwNumberOfProcessors);

	PC_t::Ptr pCloud = loadPC(ModelPath3);
	std::vector<int> Result;

	GA::CGeneticClustering GC(4, 20, 4, 250);
	GC.setCloud(pCloud);
	GC.setThreadSize(SysInfo.dwNumberOfProcessors);

	hiveCommon::CCPUTimer Timer;
	double t = Timer.getElapsedTime();
	Timer.start();
	/* Timer */
	GC.run();
	/* Timer */
	Timer.stop();
	t = Timer.getElapsedTimeInMS();
	
	GC.dumpBestResult(Result);
	_ASSERTE(Result.size() == pCloud->size());

	PC_t::Ptr pOutput(new PC_t);
	for (int i = 0; i < 8; i++)
	{
		PC_t::Ptr pPart(new PC_t);
		for (int k = 0; k < pCloud->size(); k++)
			if (Result[k] == i)
				pPart->emplace_back(pCloud->at(k));

		if (pPart->size() == 0)
		{
			std::cout << "Cluster Numbers: " << i << std::endl;
			break;
		}

		for (auto& e : *pPart)
		{
			e.r = 0;
			e.g = 0;
			e.b = 0;

			if (i == 0)
				e.r = 255;
			else if (i == 1)
				e.g = 255;
			else if (i == 2)
				e.b = 255;
			else if (i == 3)
			{
				e.r = 255;
				e.g = 255;
			}
			else if (i == 4)
			{
				e.b = 255;
				e.g = 255;
			}
			else if (i == 5)
			{
				e.b = 255;
				e.r = 255;
			}

			pOutput->emplace_back(e);
		}
	}

	pcl::io::savePLYFileBinary("Output3.ply", *pOutput);

	std::cout << "Use Time: " << t << "ms" << std::endl;
}