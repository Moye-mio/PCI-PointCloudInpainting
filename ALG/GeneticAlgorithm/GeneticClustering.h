#pragma once

namespace GA
{
	class CCluster;

	class CGeneticClustering
	{
	public:
		CGeneticClustering(float vK, int vSolutionSize, int vClusterSize, int vLoopSize);
		~CGeneticClustering() {}

		[[nodiscard]] bool setCloud(const PC_t::Ptr& vCloud);
		[[nodiscard]] bool setNormals(const NormalPC_t::Ptr& vNormals);
		[[nodiscard]] bool run();
		void dumpBestResult(std::vector<int>& voResult);

	private:
		void __init(std::vector<std::vector<int>>& voSolutions);
		void __loop(const std::vector<std::vector<int>>& vSolutions, bool vMutateAll);
		void __calcSortedExpectedValues(const std::vector<std::vector<int>>& vSolutions, std::vector<std::pair<int, float>>& voExpectedValues);
		void __calcPointFitness(const std::vector<int>& vSolution, std::vector<float>& vPointFitnessInSolution);
		void __clearRedundantClusters(std::vector<std::vector<int>>& vioSolutions);
		float __calcVolume(const PC_t::Ptr& vCloud);
		float __calcSolutionFitness(const std::vector<int>& vSolution);
		float __generateClusters(const std::vector<int>& vSolution, std::vector<CCluster>& voClusters);
		float __calcMutatePR(const std::vector<float>& vFitness, int vCate);

	private:
		std::vector<std::vector<int>>	m_ClusterResult;
		PC_t::Ptr						m_Cloud;
		NormalPC_t::Ptr					m_Normals;

		/* Parameters */
		Eigen::Vector2f		m_OperatorRate;

		float				m_Volume;
		float				m_K;
		float				m_NormalDist;

		int					m_SolutionSize;
		int					m_ClusterSize;
		int					m_DataSize;
		int					m_LoopSize;

	};
}