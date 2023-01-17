#pragma once
#include <Eigen/Eigen>

namespace core
{
	struct SPoint : public Eigen::Vector3f
	{
		SPoint() = default;
		SPoint(const SPoint&) = default;
		SPoint(SPoint&&) = default;
		SPoint& operator=(const SPoint&) = default;
		SPoint& operator=(SPoint&&) = default;

		SPoint(float vX, float vY, float vZ)
		{
			m_storage.data()[0] = vX;
			m_storage.data()[1] = vY;
			m_storage.data()[2] = vZ;
		}

		SPoint(const Eigen::Vector3f& vData)
		{
			SPoint(vData[0], vData[1], vData[2]);
		}

		SPoint(Eigen::Vector3f& vData)
		{
			SPoint(vData[0], vData[1], vData[2]);
		}

		const bool isValid() const
		{
			if (std::isnan(this->x()) || std::isnan(this->y()) || std::isnan(this->z()))
				return false;
			else
				return true;
		}
	};
}