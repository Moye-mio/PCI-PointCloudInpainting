#pragma once
#include "PointCloudType.h"

struct SOBB
{
	SOBB() = default;
	SOBB(const Eigen::Vector3f& vMin, const Eigen::Vector3f& vMax, const Eigen::Vector3f& vPos, const Eigen::Matrix3f& vRotation)
		: _Min(vMin)
		, _Max(vMax)
		, _Pos(vPos)
		, _Rotation(vRotation) {}

	Eigen::Vector3f _Min;
	Eigen::Vector3f _Max;
	Eigen::Vector3f _Pos;
	Eigen::Matrix3f _Rotation;
};
