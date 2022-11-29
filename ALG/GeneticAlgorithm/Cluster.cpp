#include "pch.h"

#include "Cluster.h"
#include "PlaneFitting.h"
#include "GaussianDistribution1D.h"
#include "GaussianDistribution2D.h"

using namespace GA;

CCluster::CCluster()
	: m_pCloud(new PC_t)
	, m_Normals(new NormalPC_t)
	, m_RansacDist(0.3f)
	, m_Plane(common::SPlane(0.0f, 0.0f, 0.0f, 0.0f))
{}

bool CCluster::setData(const PC_t::Ptr& vCloud, const NormalPC_t::Ptr& vNormals)
{
	_ASSERTE(vCloud != NULL && vNormals != NULL);
	_ASSERTE(vCloud->size() && vCloud->size() == vNormals->size());

	m_pCloud = vCloud;
	m_Normals = vNormals;

	return true;
}

bool CCluster::calcGD()
{
	__fitPlane();
	_ASSERTE(m_Plane.isValid());

	m_DistGD = std::make_shared<core::CGaussianDistribution1D>();
	m_AngleGD = std::make_shared<core::CGaussianDistribution1D>();
	m_ProjGD = std::make_shared<core::CGaussianDistribution2D>();

	__calcDistGD();
	__calcAngleGD();
	__calcProjGD();

	_ASSERTE(m_DistGD->isValid() && m_AngleGD->isValid() && m_ProjGD->isValid());

	return true;
}

float CCluster::computePointFitness(const Point_t& vPoint, const Normal_t& vNormal)
{
	_ASSERTE(m_Rotation.data());

	float Divisor = std::sqrt(m_Plane[0] * m_Plane[0] + m_Plane[1] * m_Plane[1] + m_Plane[2] * m_Plane[2]);
	Eigen::Vector3f Pos = m_Rotation * vPoint.getVector3fMap();
	float DistFitness = m_DistGD->computeProbability(__calcPointDist2Plane(vPoint, Divisor));
	float AngleFitness = m_AngleGD->computeProbability(__calcNormalAngle(vNormal, Divisor));
	float ProjFitness = m_ProjGD->computeProbability(Pos[0], Pos[1]);

	_ASSERTE(__isFitnessValid(DistFitness) && __isFitnessValid(AngleFitness) && __isFitnessValid(ProjFitness));

	return DistFitness * AngleFitness * ProjFitness;
}

void CCluster::__fitPlane()
{
	if (m_pCloud->size() == 1)
	{
		m_Plane[2] = 1.0f;
		m_Plane[3] = -m_pCloud->at(0).z;
	}
	else
	{
		bool IsErase = false;
		if (m_pCloud->size() == 2)
		{
			Eigen::Vector3f Dir = m_pCloud->at(0).getVector3fMap() - m_pCloud->at(1).getVector3fMap();
			_ASSERTE(!(Dir[0] == 0.0f && Dir[1] == 0.0f && Dir[2] == 0.0f));

			if (Dir[0] == Dir[1] && Dir[0] == Dir[2])
				Dir[0] += Dir[1];
			else
				for (int i = 0; i < Dir.size() - 1; i++)
					if (Dir[i] != Dir[i + 1])
					{
						float Temp = Dir[i];
						Dir[i] = Dir[i + 1];
						Dir[i + 1] = Temp;
						break;
					}

			m_pCloud->emplace_back(Point_t(Dir[0] + m_pCloud->at(0).x, Dir[1] + m_pCloud->at(0).y, Dir[2] + m_pCloud->at(0).z));
			IsErase = true;
		}

		core::CPlaneFitting Fitting;
		m_Plane = Fitting.fitRansacPlane(m_pCloud, m_RansacDist);

		if (IsErase)
			m_pCloud->erase(m_pCloud->end() - 1);
	}
}

void CCluster::__calcDistGD()
{
	float Divisor = std::sqrt(m_Plane[0] * m_Plane[0] + m_Plane[1] * m_Plane[1] + m_Plane[2] * m_Plane[2]);
	std::vector<float> Dists;
	for (auto& e : *m_pCloud)
		Dists.push_back(__calcPointDist2Plane(e, Divisor));

	m_DistGD->setData(Dists);
}

void CCluster::__calcAngleGD()
{
	float Divisor = std::sqrt(m_Plane[0] * m_Plane[0] + m_Plane[1] * m_Plane[1] + m_Plane[2] * m_Plane[2]);
	std::vector<float> Angles;

	//std::cout << "-------------SinValue-------------" << std::endl;

	for (int i = 0; i < m_pCloud->size(); i++)
		Angles.push_back(__calcNormalAngle(m_Normals->at(i), Divisor));

	m_AngleGD->setData(Angles);
}

void CCluster::__calcProjGD()
{
	__calcRotation();
	_ASSERTE(m_Rotation.data());

	std::vector<float> Proj1, Proj2;
	for (auto& e : *m_pCloud)
	{
		Eigen::Vector3f Pos = m_Rotation * e.getVector3fMap();
		Proj1.push_back(Pos[0]);
		Proj2.push_back(Pos[1]);
	}

	m_ProjGD->setData(Proj1, Proj2);
}

void CCluster::__calcRotation()
{
	// 求Plane和z轴的交点，以该交点为旋转点，交点处法线为z'轴，y=0和平面交线为x'轴，建立临时坐标系，目标为将临时坐标系绕旋转点旋转到新坐标系
	float Divisor = std::sqrtf(m_Plane[0] * m_Plane[0] + m_Plane[1] * m_Plane[1] + m_Plane[2] * m_Plane[2]);
	_ASSERTE(Divisor > 0);

	Eigen::Matrix3f Rotation;
	if (m_Plane[2] != 0)
	{
		float InnerProduct = m_Plane[2];
		float SinAngle = std::fabsf(InnerProduct) / (Divisor * std::fabsf(m_Plane[2]));
		SinAngle = std::clamp(SinAngle, 0.0f, 1.0f);
		float CosAngle = std::sqrt(1 - SinAngle * SinAngle);

		Rotation << CosAngle, SinAngle, 0,
			-SinAngle, CosAngle, 0,
			0, 0, 1;
	}
	else
	{
		float Angle = 90 * std::numbers::pi / 180;
		Eigen::Vector3f Axis(-m_Plane[1] / Divisor, m_Plane[0] / Divisor, 0);
		float K = 1 - std::cos(Angle);

		Rotation << Axis[0] * Axis[0] * K + std::cos(Angle), Axis[0] * Axis[1] * K, Axis[1] * std::sin(Angle),
			Axis[0] * Axis[1] * K, Axis[1] * Axis[1] * K + std::cos(Angle), -Axis[0] * std::sin(Angle),
			-Axis[1] * std::sin(Angle), Axis[0] * std::sin(Angle), std::cos(Angle);
	}
	m_Rotation = Rotation;
}

bool CCluster::__isFitnessValid(float vFitness)
{
	if (std::isnan(vFitness))
		return false;
	if (vFitness < 0)
		return false;
	return true;
}

float CCluster::__calcPointDist2Plane(const Point_t& vPoint, float vDivisor /*= 0.0f*/)
{
	if (vDivisor == 0.0f)
		vDivisor = std::sqrt(m_Plane[0] * m_Plane[0] + m_Plane[1] * m_Plane[1] + m_Plane[2] * m_Plane[2]);
	return std::fabsf(m_Plane[0] * vPoint.x + m_Plane[1] * vPoint.y + m_Plane[2] * vPoint.z + m_Plane[3]) / vDivisor;
}

float CCluster::__calcNormalAngle(const Normal_t& vNormal, float vDivisor /*= 0.0f*/)
{
	float InnerProduct = vNormal.normal_x * m_Plane[0] + vNormal.normal_y * m_Plane[1] + vNormal.normal_z * m_Plane[2];
	float PointNormal = std::sqrt(vNormal.normal_x * vNormal.normal_x + vNormal.normal_y * vNormal.normal_y + vNormal.normal_z * vNormal.normal_z);
	return std::fabsf(InnerProduct) / vDivisor * PointNormal;
}
