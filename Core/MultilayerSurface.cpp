#include "pch.h"

#include "MultilayerSurface.h"
#include "BSplineSurface.h"

using namespace core;

//CMultilayerSurface::CMultilayerSurface(int vDegree, bool vIsClamped /*= true*/)
//	: IBSplineSurface(vDegree)
//	, m_SubNum(2)
//	, m_SubLayer(5)
//	, m_IsCalcError(true)
//{}
//
//bool CMultilayerSurface::setSubNumber(int vSubNum)
//{
//	_ASSERTE(vSubNum > 0);
//	m_SubNum = vSubNum;
//	return true;
//}
//
//bool CMultilayerSurface::setSubLayer(int vSubLayer)
//{
//	_ASSERTE(vSubLayer > 0);
//	m_SubLayer = vSubLayer;
//	return true;
//}
//
//bool CMultilayerSurface::setIsCalcError(bool vIsCalcError)
//{
//	m_IsCalcError = vIsCalcError;
//	return true;
//}
//
//std::optional<core::SProjInfo> CMultilayerSurface::calcProj(const SPoint& vPoint)
//{
//	_ASSERTE(vPoint.isValid());
//
//	return std::nullopt;
//}
//
//std::optional<float> CMultilayerSurface::__HitTriangle(const CTriangle& vTri, const SPoint& vPoint)
//{
//	common::SPlane Plane;
//	vTri.calcPlane(Plane, false);
//	Eigen::Vector3f ProjRay;
//	float Dist = Plane.calcPointProject(vPoint, ProjRay);
//	_ASSERTE(!std::isnan(Dist));
//	bool r = vTri.isRayIntersection(vPoint, ProjRay);
//	if (r == false)
//		return std::nullopt;
//	else
//		return Dist;
//}
//
//float CMultilayerSurface::__calcPointDist(const SPoint& vLhs, const SPoint& vRhs)
//{
//	_ASSERTE(vLhs.isValid() && vRhs.isValid());
//	return (vLhs - vRhs).norm();
//}
//
