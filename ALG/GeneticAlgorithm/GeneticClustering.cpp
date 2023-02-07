#include "pch.h"

#include "GeneticClustering.h"
#include "OBBPCA.h"
#include "NormalEstimator.h"
#include "Cluster.h"

using namespace GA;

CGeneticClustering::CGeneticClustering(float vK, int vSolutionSize, int vClusterSize, int vLoopSize)
	: m_K(vK)
	, m_SolutionSize(vSolutionSize)
	, m_ClusterSize(vClusterSize)
	, m_LoopSize(vLoopSize)
	, m_DataSize(0)
	, m_ThreadSize(4)
	, m_Volume(0.0f)
	, m_NormalDist(0.3f)
	, m_OperatorRate({0.3f, 0.6f})
	, m_Cloud(new PC_t)
	, m_Normals(new NormalPC_t)
{}

bool CGeneticClustering::setThreadSize(int vSize)
{
	_ASSERTE(vSize >= 4);
	m_ThreadSize = vSize;
	return true;
}

bool CGeneticClustering::setCloud(const PC_t::Ptr& vCloud)
{
	_ASSERTE(vCloud != NULL);
	_ASSERTE(vCloud->size());

	if (m_Normals->size())
		_ASSERTE(m_Normals->size() == vCloud->size());

	m_Cloud = vCloud;
	m_DataSize = vCloud->size();

	return true;
}

bool CGeneticClustering::setNormals(const NormalPC_t::Ptr& vNormals)
{
	_ASSERTE(vNormals != NULL);
	_ASSERTE(vNormals->size());

	if (m_Cloud->size())
		_ASSERTE(m_Cloud->size() == vNormals->size());

	m_Normals = vNormals;

	return true;
}

bool CGeneticClustering::run()
{
	_ASSERTE(m_DataSize);
	
	std::vector<std::vector<int>> Solutions;	// SolutionSize �� DataSize
	__init(Solutions);
	_ASSERTE(m_Volume && m_Normals->size() == m_DataSize);

	for (int i = 0; i < m_LoopSize; i++)
	{
		if (i > 0)
			Solutions = m_ClusterResult;

		std::cout << "Loop " << i << ": " << std::endl;

		if (i < m_LoopSize / 2)
			__loop(Solutions, false);
		else
 			__loop(Solutions, true);

		__clearRedundantClusters(m_ClusterResult);
	}

	return true;
}

void CGeneticClustering::dumpBestResult(std::vector<int>& voResult)
{
	std::vector<std::pair<int, float>> ExpectedValue;
	__calcSortedExpectedValues(m_ClusterResult, ExpectedValue);

	voResult = m_ClusterResult[ExpectedValue[0].first];
}

void CGeneticClustering::__init(std::vector<std::vector<int>>& voSolutions)
{
	if (voSolutions.size())
	{
		voSolutions.clear();
		voSolutions.shrink_to_fit();
	}

	for (int i = 0; i < m_SolutionSize; i++)
		voSolutions.emplace_back(hiveMath::hiveGenerateRandomIntegerSet<int>(0, m_ClusterSize - 1, m_DataSize));

	m_Volume = __calcVolume(m_Cloud);

	if (!m_Normals->size())
	{
		core::CNormalEstimator NormalEstimator;
		NormalEstimator.setCloud(m_Cloud);
		NormalEstimator.compute(m_NormalDist);
		NormalEstimator.dumpNormals(m_Normals);
	}
}

void CGeneticClustering::__calcSortedExpectedValues(const std::vector<std::vector<int>>& vSolutions, std::vector<std::pair<int, float>>& voExpectedValues)
{
	std::vector<float> Fitness;
	for (int i = 0; i < m_SolutionSize; i++)
	{
		float Fit = __calcSolutionFitness(vSolutions[i]);
		_ASSERTE(Fit);
		Fitness.push_back(Fit);
	}

	float MeanFitness = std::accumulate(Fitness.begin(), Fitness.end(), 0.0f) / m_SolutionSize;
	for (int i = 0; i < m_SolutionSize; i++)
		voExpectedValues.emplace_back(std::make_pair(i, Fitness[i] / MeanFitness));

	Fitness.clear();
	Fitness.shrink_to_fit();

	std::sort(voExpectedValues.begin(), voExpectedValues.end(),
		[&](const std::pair<int, float>& a, const std::pair<int, float>& b) -> bool { return a.second > b.second; });

	{
		std::cout << "Fitness Sorted: \n";
		for (auto& e : voExpectedValues)
			std::cout << e.first << ", " << e.second << std::endl;
	}
}

float CGeneticClustering::__calcVolume(const PC_t::Ptr& vCloud)
{
	core::COBBPCA OBBEstimation;
	OBBEstimation.compute(vCloud);
	return OBBEstimation.computeOBBVolume();
}

float CGeneticClustering::__calcSolutionFitness(const std::vector<int>& vSolution)
{
	std::vector<float> VolumeAll;
	for (int i = 0; i < m_ClusterSize; i++)
	{
		PC_t::Ptr pCloud(new PC_t);
		int Count = -1;
		for (auto e : vSolution)
		{
			Count++;
			if (e != i) continue;
			pCloud->emplace_back(m_Cloud->at(Count));
		}

		if (pCloud->size() == 0) continue;
		VolumeAll.push_back(__calcVolume(pCloud));
	}

	return (m_K * m_Volume - std::accumulate(VolumeAll.begin(), VolumeAll.end(), 0.0f)) / std::sqrtf(VolumeAll.size());
}

void CGeneticClustering::__calcPointFitness(const std::vector<int>& vSolution, std::vector<float>& vPointFitnessInSolution)
{
	std::vector<float> PointFitnessInSolution(m_DataSize, 0);
	for (int i = 0; i < m_ClusterSize; i++)
	{
		/* Extract Cloud and Normals for Each Solution */
		PC_t::Ptr pNewCloud(new PC_t);
		NormalPC_t::Ptr pNewNormals(new NormalPC_t);
		std::vector<float> PointFitness2Cloud;

		for (int k = 0; k < m_DataSize; k++)
			if (vSolution[k] == i)
			{
				pNewCloud->emplace_back(m_Cloud->at(k));
				pNewNormals->emplace_back(m_Normals->at(k));
			}

		if (pNewCloud->size() == 0) continue;

		_ASSERTE(pNewCloud->size() == pNewNormals->size());

		CCluster Cluster;
		Cluster.setData(pNewCloud, pNewNormals);
		bool r = Cluster.calcGD();
		_ASSERTE(r);
		
		for (int i = 0; i < pNewCloud->size(); i++)
			PointFitness2Cloud.push_back(Cluster.computePointFitness(pNewCloud->at(i), pNewNormals->at(i)));

		int Count = 0;
		for (int k = 0; k < m_DataSize; k++)
			if (vSolution[k] == i)
			{
				PointFitnessInSolution[k] = PointFitness2Cloud[Count];
				Count++;
			}
	}
	vPointFitnessInSolution = PointFitnessInSolution;
}

float CGeneticClustering::__generateClusters(const std::vector<int>& vSolution, std::vector<CCluster>& voClusters)
{
	_ASSERTE(vSolution.size() == m_DataSize);

	voClusters.clear();
	voClusters.shrink_to_fit();

	for (int i = 0; i < m_ClusterSize; i++)
	{
		PC_t::Ptr pNewCloud(new PC_t);
		NormalPC_t::Ptr pNewNormals(new NormalPC_t);

		for (int k = 0; k < m_DataSize; k++)
			if (vSolution[k] == i)
			{
				pNewCloud->emplace_back(m_Cloud->at(k));
				pNewNormals->emplace_back(m_Normals->at(k));
			}

		if (pNewCloud->size() == 0) continue;

		_ASSERTE(pNewCloud->size() == pNewNormals->size());

		CCluster Cluster;
		Cluster.setData(pNewCloud, pNewNormals);
		bool r = Cluster.calcGD();
		_ASSERTE(r);

		voClusters.emplace_back(Cluster);
	}
	return true;
}

float CGeneticClustering::__calcMutatePR(const std::vector<float>& vFitness, int vCate)
{
	return (1 - vFitness[vCate] / std::accumulate(vFitness.begin(), vFitness.end(), 0.0f));
}

void CGeneticClustering::__loop(const std::vector<std::vector<int>>& vSolutions, bool vMutateAll)
{
	std::vector<std::pair<int, float>> ExpectedValue;	// Solution <index, fitness>
	__calcSortedExpectedValues(vSolutions, ExpectedValue);

	std::vector<std::vector<int>> NextSolutions;
	std::unordered_map<int, std::vector<float>> Points2Clusters;	// PCs < index, fc >
	std::mutex Mutex;

#pragma omp parallel for num_threads(m_ThreadSize)
	for (int i = 0; i < m_SolutionSize; i++)
	{
		std::vector<int> CurSolution = vSolutions[ExpectedValue[i].first];
		if (i < m_SolutionSize * m_OperatorRate[0])
		{
			{
				std::cout << "Select: " << ExpectedValue[i].first << std::endl;
			}
			Mutex.lock();
			NextSolutions.emplace_back(CurSolution);
			Mutex.unlock();
		}
		if (i < m_SolutionSize * m_OperatorRate[1])
		{
			{
				std::cout << "Cross : " << ExpectedValue[i].first << std::endl;
			}
			std::vector<float> PointFitnessInSolution;
			__calcPointFitness(CurSolution, PointFitnessInSolution);
			Points2Clusters.emplace(std::make_pair(i, PointFitnessInSolution));
		}
		
		if (vMutateAll == false && i < m_SolutionSize * m_OperatorRate[1]) continue;

		/* Mutate */
		{
			std::vector<CCluster> Clusters;
			__generateClusters(CurSolution, Clusters);
			std::vector<int> MutationSolution = CurSolution;

			int MutateNumber = 0;

			for (int k = 0; k < m_DataSize; k++)
			{
				auto& Point = m_Cloud->at(k);
				auto& Normal = m_Normals->at(k);
				std::vector<float> PointFitness;
				for (auto& e : Clusters)
					PointFitness.push_back(e.computePointFitness(Point, Normal));

				float PR = __calcMutatePR(PointFitness, CurSolution[k]);
				if (hiveMath::hiveGenerateRandomReal(0.0f, 1.0f) < PR)
				{
					MutateNumber++;
					MutationSolution[k] = std::distance(PointFitness.begin(), std::max_element(PointFitness.begin(), PointFitness.end()));
				}
			}

			{
				std::cout << "Mutate: " << ExpectedValue[i].first << "\t" << MutateNumber << std::endl;
			}

			Mutex.lock();
			NextSolutions.emplace_back(MutationSolution);
			Mutex.unlock();
		}
	}

	/* Cross */
	int RandomNumber = int(m_SolutionSize * m_OperatorRate[1] - 1);
	auto RandomSet = hiveMath::hiveGenerateNoDuplicateRandomIntegerSet(0, RandomNumber, RandomNumber + 1);
	//auto RandomSet = hiveMath::hiveGenerateNoDuplicateRandomIntegerSet(int(m_SolutionSize * m_OperatorRate[0]), int(m_SolutionSize * m_OperatorRate[1] - 1), int(m_SolutionSize * (m_OperatorRate[1] - m_OperatorRate[0])));
	hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("RandomSet Size [%1%]", RandomSet.size()));
	for (int i = 0; i < RandomSet.size(); i += 2)
	{
		int Major = (RandomSet[i] < RandomSet[i + 1]) ? RandomSet[i] : RandomSet[i + 1];
		int Minor = (RandomSet[i] >= RandomSet[i + 1]) ? RandomSet[i] : RandomSet[i + 1];

		{
			std::cout << "Cross exchange: " << ExpectedValue[Major].first << ", " << ExpectedValue[Minor].first << std::endl;
		}

		auto IterM = Points2Clusters.find(Major);
		auto Iterm = Points2Clusters.find(Minor);
		_ASSERTE(IterM != Points2Clusters.end() && Iterm != Points2Clusters.end());

		auto PCMajor = IterM->second;
		auto PCMinor = Iterm->second;
		_ASSERTE(PCMajor.size() == PCMinor.size() && PCMajor.size() == m_DataSize);

		std::vector<int> MajorCrossSolution = vSolutions[ExpectedValue[Major].first];
		for (int k = 0; k < m_DataSize; k++)
		{
			if (PCMajor[k] == PCMinor[k]) continue;

			float Random = hiveMath::hiveGenerateRandomReal(0.0f, 1.0f);
			float MajorConfidence = PCMajor[k] / (PCMajor[k] + PCMinor[k]);
			if (Random > MajorConfidence)
				MajorCrossSolution[k] = vSolutions[ExpectedValue[Minor].first][k];
		}
		NextSolutions.emplace_back(MajorCrossSolution);
	}

	m_ClusterResult = NextSolutions;
}

void CGeneticClustering::__clearRedundantClusters(std::vector<std::vector<int>>& vioSolutions)
{
	for (auto& e : vioSolutions)
	{
		std::vector<int> ClusterPointCount(m_ClusterSize, 0);
		for (auto p : e)
			ClusterPointCount[p]++;

		std::vector<int> CateMapping;
		for (int i = 0; i < m_ClusterSize; i++)
			CateMapping.push_back(i);

		for (int i = 0; i < ClusterPointCount.size(); i++)
			if (ClusterPointCount[i] == 0)
			{
				CateMapping[i] = -1;
				for (int k = i + 1; k < CateMapping.size(); k++)
					CateMapping[k] -= 1;
			}

		for (int i = 0; i <m_DataSize; i++)
		{
			_ASSERTE(CateMapping[e[i]] >= 0);
			if (e[i] != CateMapping[e[i]])
				e[i] = CateMapping[e[i]];
		}
	}
}
