#pragma once
#include "BSplineSurface.h"

namespace core
{
	struct SProjInfo
	{
		SPoint			_Point;
		Eigen::Vector2f _UV;
		float			_Dist;
	};

	struct SVertex
	{
		float x, y, z;
		float u, v;

		SVertex()
			: x(0), y(0), z(0), u(0), v(0) {}

		SVertex(float vX, float vY, float vZ)
			: x(vX), y(vY), z(vZ) {}

		SVertex(float vX, float vY, float vZ, float vU, float vV)
			: x(vX), y(vY), z(vZ), u(vU), v(vV) {}

		const bool isValid() const
		{
			if (std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(u) || std::isnan(v))	return false;
			else																					return true;
		}
	};

	class CMultilayerSurface : public IBSplineSurface
	{
	public:
		CMultilayerSurface(int vDegree, bool vIsClamped = true);

		[[nodiscard]] bool setSubNumber(int vSubNum);
		[[nodiscard]] bool setSubLayer(int vSubLayer);
		[[nodiscard]] bool setIsCalcError(bool vIsCalcError);

		std::optional<SProjInfo> calcProj(const SPoint& vPoint);

	private:
		bool __preCompute();
		bool __IsComputed();
		void __calcRange(const std::vector<Eigen::Vector2i>& vHit, std::pair<Eigen::Vector2i, Eigen::Vector2i>& voRange);
		std::optional<SVertex>		__transPoint2Vertex(const SPoint& vPoint, const Eigen::Vector2f vUV = Eigen::Vector2f(0, 0));
		std::optional<SVertex>		__sample(const Eigen::Matrix<SVertex, -1, -1>& vNodes, float vU, float vV);
		std::optional<SPoint>		__transVertex2Point(const SVertex& vVertex);
		std::optional<CTriangle>	__geneTriangle(const SVertex& vP1, const SVertex& vP2, const SVertex& vP3);
		std::optional<float>		__HitTriangle(const CTriangle& vTri, const SPoint& vPoint);
		std::optional<float>		__calcPointDist(const SPoint& vLhs, const SPoint& vRhs);
		std::optional<float>		__calcHitNodes(int vLayer, const SPoint& vPoint, const std::pair<Eigen::Vector2i, Eigen::Vector2i>& vRange, std::vector<Eigen::Vector2i>& voHit);

	private:
		std::vector<Eigen::Matrix<SVertex, -1, -1>>		m_Vertices;
		std::vector<std::vector<Eigen::Vector3i>>		m_Triangles;

		int		m_SubNum;
		int		m_SubLayer;
		bool	m_IsCalcError;
	};
}