#include "pch.h"
#include "Surface2PCMapper.h"

#include "HeightMap.h"

std::optional<Point_t> core::CSurface2PCMapper::generatePoint(const SVertex& vProjPoint, const Eigen::Vector3f& vNormal, float vDist)
{
	_HIVE_EARLY_RETURN(vProjPoint.isValid() == false, "ERROR: CSurface2PCMapper: Project Point is no Valid", std::nullopt);
	_HIVE_EARLY_RETURN(__isNormalValid(vNormal) == false, "ERROR: CSurface2PCMapper: Normal is no Valid", std::nullopt);
	_HIVE_EARLY_RETURN(std::isnan(vDist), "ERROR: CSurface2PCMapper: Dist is NULL", std::nullopt);

	Eigen::Vector3f PointPos(vProjPoint.x, vProjPoint.y, vProjPoint.z);
	PointPos += vNormal * vDist;
	
	return Point_t(PointPos.x(), PointPos.y(), PointPos.z());
}

bool core::CSurface2PCMapper::__isNormalValid(const Eigen::Vector3f& vNormal)
{
	if (std::isnan(vNormal.x()) || std::isnan(vNormal.y()) || std::isnan(vNormal.z()))
		return false;
	
	float Epsilon = 0.0001f;
	if (std::fabsf(vNormal.norm() - 1.0f) >= Epsilon)
		return false;

	return true;
}
