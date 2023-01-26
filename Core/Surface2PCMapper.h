#pragma once

namespace core
{
	class CSurface2PCMapper
	{
	public:
		CSurface2PCMapper() {}
		~CSurface2PCMapper() {}

		std::optional<Point_t> generatePoint(const SVertex& vProjPoint, const Eigen::Vector3f& vNormal, float vDist);

	private:
		bool __isNormalValid(const Eigen::Vector3f& vNormal);
		bool __normalizeNormal(Eigen::Vector3f& vioNormal);
	};
}