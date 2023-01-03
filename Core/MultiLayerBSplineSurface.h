#pragma once
#include "BSplineSurface.h"

namespace core
{
	class CMultiLayerBSplineSurface : IBSplineSurface
	{
	public:
		[[nodiscard]] bool setLayer(int vLayer);
		float calcProj(const SPoint& vPoint, Eigen::Vector2f& voUV);

	private:
		void __generateMultiLayerNodes();
		bool __isMultiLayerReady();
		bool __calcUV(const CTriangle& vTri, const SPoint& vPoint, Eigen::Vector2f& voUV);
		std::optional<Eigen::Vector2f> __findUV(const SPoint& vPoint);
		std::optional<float> __isHitNodes(const Eigen::Matrix<SPoint, -1, -1>& vNodes, const SPoint& vPoint, CTriangle& voTri);
		std::optional<float> __isHitTriangle(const CTriangle& vTri, const SPoint& vPoint);

	private:
		int m_Sub = 5;
		int m_Layers = 3;

		Eigen::Matrix<Eigen::Vector2f, -1, -1>		m_LatestLayerUV;
		std::vector<Eigen::Matrix<SPoint, -1, -1>>	m_MultiLayerNodes;
	};
}