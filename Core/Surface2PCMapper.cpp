#include "pch.h"
#include "Surface2PCMapper.h"

#include "HeightMap.h"

std::optional<Point_t> core::CSurface2PCMapper::generatePoint(const SVertex& vProjPoint, const Eigen::Vector3f& vNormal, float vDist)
{
	Eigen::Vector3f Normal = vNormal;

	_HIVE_EARLY_RETURN(vProjPoint.isValid() == false, "ERROR: CSurface2PCMapper: Project Point is not Valid", std::nullopt);
	_HIVE_EARLY_RETURN(__normalizeNormal(Normal) == false, _FORMAT_STR3("ERROR: CSurface2PCMapper: Normal is not Valid [%1%, %2%, %3%]", vNormal[0], vNormal[1], vNormal[2]), std::nullopt);
	_HIVE_EARLY_RETURN(std::isnan(vDist), "ERROR: CSurface2PCMapper: Dist is NULL", std::nullopt);

	Eigen::Vector3f PointPos(vProjPoint.x, vProjPoint.y, vProjPoint.z);
	PointPos += Normal * vDist;
	
	return Point_t(PointPos.x(), PointPos.y(), PointPos.z());
}

bool core::CSurface2PCMapper::__isNormalValid(const Eigen::Vector3f& vNormal)
{
	_HIVE_EARLY_RETURN(std::isnan(vNormal.x()) || std::isnan(vNormal.y()) || std::isnan(vNormal.z()), "Normal has nan Value", false);
	return true;
}

bool core::CSurface2PCMapper::__normalizeNormal(Eigen::Vector3f& vioNormal)
{
	if (__isNormalValid(vioNormal) == false)
		return false;

	float Epsilon = 0.0001f;
	if (std::fabsf(vioNormal.norm() - 1.0f) >= Epsilon)
	{
		float SumOfSquares = vioNormal[0] * vioNormal[0] + vioNormal[1] * vioNormal[1] + vioNormal[2] * vioNormal[2];
		if (SumOfSquares == 0.0f)
			return false;

		vioNormal /= std::sqrtf(SumOfSquares);
	}
	
	return true;
}
