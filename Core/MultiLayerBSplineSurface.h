#pragma once
#include "BSplineSurface.h"

namespace core
{
	class CMultiLayerBSplineSurface : public IBSplineSurface
	{
	public:
		CMultiLayerBSplineSurface(int vDegree, bool vIsClamped = true);
		[[nodiscard]] bool setLayer(int vLayer);
		float calcProj(const SPoint& vPoint, Eigen::Vector2f& voUV);
		std::optional<SPoint> getDetailedNode(int vRow, int vCol);

	private:
		void __generateMultiLayerNodes();
		bool __isMultiLayerReady();
		bool __calcUV(const CTriangle& vTri, const SPoint& vPoint, Eigen::Vector2f& voUV);
		std::optional<Eigen::Vector2f> __findUV(const SPoint& vPoint);
		std::optional<float> __isHitNodes(const Eigen::Matrix<SPoint, -1, -1>& vNodes, const SPoint& vPoint, CTriangle& voTri);
		std::optional<float> __isHitTriangle(const CTriangle& vTri, const SPoint& vPoint);

	private:
		int m_Sub;
		int m_Layers;

		Eigen::Matrix<Eigen::Vector2f, -1, -1>		m_LatestLayerUV;
		std::vector<Eigen::Matrix<SPoint, -1, -1>>	m_MultiLayerNodes;
	};
}