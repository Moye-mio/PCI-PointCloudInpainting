#pragma once
#include <Eigen/Eigen>

namespace core
{
	struct SPoint : public Eigen::Vector3f
	{
		const bool isValid() const
		{
			if (std::isnan(this->x()) || std::isnan(this->y()) || std::isnan(this->z()))
				return false;
			else
				return true;
		}
	};
}