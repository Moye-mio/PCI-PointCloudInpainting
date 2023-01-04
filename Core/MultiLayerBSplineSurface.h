#pragma once
#include "BSplineSurface.h"

namespace core
{
	class CMultiLayerBSplineSurface : public IBSplineSurface
	{
	public:
		CMultiLayerBSplineSurface(int vDegree, bool vIsClamped = true);
		[[nodiscard]] bool setLayer(int vLayer);
		[[nodiscard]] bool setMaxSub(int vMaxSub);
		float calcProj(const SPoint& vPoint, Eigen::Vector2f& voUV);
		std::optional<SPoint> getDetailedNode(int vRow, int vCol);

	private:
		void __generatePreMultiLayerNodes();
		bool __isMultiLayerReady();
		std::optional<Eigen::Vector3f> __calcBaryCoor(const Eigen::Matrix<SPoint, -1, -1>& vNodes, const std::vector<Eigen::Vector2i>& vHit, const SPoint& vPoint);
		Eigen::Vector2f __calcUV(const std::vector<Eigen::Vector2f>& vUV, const Eigen::Vector3f& vBary);
		float __calcNodesDiff(const SPoint& vLhs, const SPoint& vRhs);
		Eigen::Vector2i __calcStartIndices(const std::vector<Eigen::Vector2i>& vHit, int vRow, int vCol);
		void __extractLocalNodes(const Eigen::Matrix<SPoint, -1, -1>& vNodes, const Eigen::Vector2i& vStart, Eigen::Matrix<SPoint, -1, -1>& vLocal);
		void __extractLocalUV(const Eigen::Matrix<Eigen::Vector2f, -1, -1>& vUV, const Eigen::Vector2i& vStart, std::pair<Eigen::Vector2f, Eigen::Vector2f>& vioUV);
		void __subdivide(const Eigen::Matrix<SPoint, -1, -1>& vRough, Eigen::Matrix<SPoint, -1, -1>& voSub, Eigen::Matrix<Eigen::Vector2f, -1, -1>& voUV);
		std::optional<Eigen::Vector2i> __findNodes(const Eigen::Matrix<SPoint, -1, -1>& vNodes, const SPoint& vPoint);
		std::optional<float> __HitNodes(const Eigen::Matrix<SPoint, -1, -1>& vNodes, const SPoint& vPoint, std::vector<Eigen::Vector2i>& voHit);
		std::optional<float> __HitTriangle(const CTriangle& vTri, const SPoint& vPoint);

	private:
		int m_Sub;
		int m_PreLayers;
		int m_MaxSub;

		Eigen::Matrix<Eigen::Vector2f, -1, -1>		m_PreComputeUV;
		std::vector<Eigen::Matrix<SPoint, -1, -1>>	m_PreComputeNodes;
	};
}