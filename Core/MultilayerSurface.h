#pragma once

namespace core
{
	struct SProjInfo
	{
		SPoint			_Point;
		Eigen::Vector2f _UV;
		float			_Dist;
	};

	//class CMultilayerSurface : public IBSplineSurface
	//{
	//public:
	//	CMultilayerSurface(int vDegree, bool vIsClamped = true);
	//	//~CMultilayerSurface() = default;

	//	[[nodiscard]] bool setSubNumber(int vSubNum);
	//	[[nodiscard]] bool setSubLayer(int vSubLayer);
	//	[[nodiscard]] bool setIsCalcError(bool vIsCalcError);

	//	std::optional<SProjInfo> calcProj(const SPoint& vPoint);

	//private:
	//	std::optional<float> __HitTriangle(const CTriangle& vTri, const SPoint& vPoint);
	//	float __calcPointDist(const SPoint& vLhs, const SPoint& vRhs);

	//private:
	//	int		m_SubNum;
	//	int		m_SubLayer;
	//	bool	m_IsCalcError;
	//};
}