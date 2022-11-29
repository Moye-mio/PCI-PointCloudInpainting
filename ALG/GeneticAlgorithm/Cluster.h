#pragma once

#include "Plane.h"

namespace core
{
	class CGaussianDistribution1D;
	class CGaussianDistribution2D;
}

namespace GA
{
	class CCluster
	{
	public:
		CCluster();
		~CCluster() {}

		[[nodiscard]] bool setData(const PC_t::Ptr& vCloud, const NormalPC_t::Ptr& vNormals);
		[[nodiscard]] bool calcGD();
		float computePointFitness(const Point_t& vPoint, const Normal_t& vNormal);

	private:
		bool __isFitnessValid(float vFitness);
		void __fitPlane();
		void __calcDistGD();
		void __calcAngleGD();
		void __calcProjGD();
		void __calcRotation();
		float __calcPointDist2Plane(const Point_t& vPoint, float vDivisor = 0.0f);
		float __calcNormalAngle(const Normal_t& vNormal, float vDivisor = 0.0f);

	private:
		PC_t::Ptr			m_pCloud;
		NormalPC_t::Ptr		m_Normals;
		common::SPlane		m_Plane;
		Eigen::Matrix3f		m_Rotation;

		std::shared_ptr<core::CGaussianDistribution1D> m_DistGD;
		std::shared_ptr<core::CGaussianDistribution1D> m_AngleGD;
		std::shared_ptr<core::CGaussianDistribution2D> m_ProjGD;

		float				m_RansacDist;
	};
}