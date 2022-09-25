#pragma once
#include "HeightMap.h"
#include "GradientMap.h"

namespace dataManagement
{
	class CSolverBuilder
	{
	public:
		CSolverBuilder() {}
		~CSolverBuilder() = default;

		bool setUp(const core::CHeightMap& vRaw, const core::CGradientMap& vGoG, const core::CGradientMap& vGradient);
		void dumpMatrix(Eigen::MatrixXf& voCoeff, Eigen::MatrixXf& voConst) { voCoeff = m_Coeff; voConst = m_Const; }
		void dumpUnknowns(std::vector<Eigen::Vector2f>& voUnknowns) { voUnknowns = m_Unknowns; }

	private:
		void __init();
		void __buildMask();
		void __countUnknowns();
		void __initMatrix();
		void __setUpEachEquation(int vIndex);
		std::optional<int> __findUnknownIndex(const Eigen::Vector2f& vPos);
		float __handleNeighbor(const Eigen::Vector2f& vPos, float vConstNumber, int vIndex, float vCoeff);

	private:
		core::CHeightMap	m_Raw;
		core::CHeightMap	m_Mask;
		core::CGradientMap	m_Gradient;
		core::CGradientMap	m_GoG;

		Eigen::MatrixXf		m_Coeff;
		Eigen::MatrixXf		m_Const;

		std::vector<Eigen::Vector2f> m_Unknowns;
	};
}