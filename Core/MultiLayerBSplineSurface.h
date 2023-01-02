#pragma once
#include "BSplineSurface.h"

namespace core
{
	class CMultiLayerBSplineSurface : CBSplineSurface
	{
	public:
		
		float calcProj(const SPoint& vPoint, Eigen::Vector2f& vUV);

	private:
		void __generateMultiLayerNodes();
		bool __isMultiLayerReady();
		std::optional<float> __isHitNodes(const Eigen::Matrix<SPoint, -1, -1>& vNodes, const SPoint& vPoint, CTriangle& voTri);
		std::optional<float> __isHitTriangle(const CTriangle& vTri, const SPoint& vPoint);

	private:
		int m_Sub = 5;
		int m_Layers = 3;

		std::vector<Eigen::Matrix<SPoint, -1, -1>> m_MultiLayerNodes;
	};
}