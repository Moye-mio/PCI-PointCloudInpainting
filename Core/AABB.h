#pragma once
#include <Eigen/Eigen>

namespace core
{
	struct SAABB
	{
		SAABB() = default;
		SAABB(const Eigen::Vector3f& vMin, const Eigen::Vector3f& vMax)
			: _Min(vMin)
			, _Max(vMax) {}

		[[nodiscard]] bool isValid() const 
		{
			if (std::isnan(_Min[0] + _Min[1] + _Min[2] + _Max[0] + _Max[1] + _Max[2])) return false;
			if (_Min[0] > _Max[0] && _Min[1] > _Max[1] && _Min[2] > _Max[2]) return false;
			return true;
		}

		Eigen::Vector3f _Max = { -FLT_MAX, -FLT_MAX , -FLT_MAX };
		Eigen::Vector3f _Min = { FLT_MAX, FLT_MAX , FLT_MAX };
	};
}