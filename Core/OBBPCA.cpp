#include "OBBPCA.h"
#include <Eigen/src/Eigenvalues/EigenSolver.h>
#include <pcl/common/centroid.h>

using namespace core;

void COBBPCA::compute(const PC_t::Ptr& vCloud)
{

	Eigen::Matrix3f CovarianceMatrix;
	Eigen::Vector4f Centroid;
	pcl::compute3DCentroid(*vCloud, Centroid);
	pcl::computeCovarianceMatrix(*vCloud, Centroid, CovarianceMatrix);
	CovarianceMatrix = CovarianceMatrix / vCloud->size();

	Eigen::EigenSolver<Eigen::Matrix3f> EigenMat(CovarianceMatrix);
	Eigen::Vector3f EigenValue = EigenMat.pseudoEigenvalueMatrix().diagonal();
	Eigen::Matrix3f EigenVector = EigenMat.pseudoEigenvectors();

	std::vector<std::tuple<float, Eigen::Vector3f>> EigenValueAndVector;
	int Size = static_cast<int>(EigenValue.size());
	EigenValueAndVector.reserve(Size);

	/*if (EigenVector.col(0).cross(EigenVector.col(1)).normalized().dot(EigenVector.col(2)) < 0)
		EigenVector.col(2).swap(-EigenVector.col(2));*/

	for (int i = 0; i < Size; ++i)
		EigenValueAndVector.push_back(std::tuple<float, Eigen::Vector3f>(abs(EigenValue[i]), EigenVector.col(i)));
	std::sort(EigenValueAndVector.begin(), EigenValueAndVector.end(),
		[&](const std::tuple<float, Eigen::Vector3f>& a, const std::tuple<float, Eigen::Vector3f>& b) -> bool
		{
			return std::get<0>(a) > std::get<0>(b);
		});
	for (int i = 0; i < Size; ++i)
	{
		//EigenVector.col(i).swap(std::get<1>(EigenValueAndVector[i]));
		for (int k = 0; k < Size; k++)
			EigenVector(i, k) = std::get<1>(EigenValueAndVector[i])[k];
	}

	Eigen::Vector3f Min{ FLT_MAX, FLT_MAX, FLT_MAX };
	Eigen::Vector3f Max{ -FLT_MAX, -FLT_MAX, -FLT_MAX };
	auto update = [&](const Eigen::Vector3f& vPos)
	{
		for (int i = 0; i < 3; i++)
		{
			if (vPos.data()[i] < Min.data()[i])
				Min.data()[i] = vPos.data()[i];
			if (vPos.data()[i] > Max.data()[i])
				Max.data()[i] = vPos.data()[i];
		}
	};

	const Eigen::Vector3f Center(Centroid.x(), Centroid.y(), Centroid.z());
	for (const auto& Pos : *vCloud)
	{
		Eigen::Vector3f AfterPos = EigenVector * (Pos.getVector3fMap() - Center) + Center;
		update(AfterPos);
	}

	m_OBB = SOBB(Min, Max, Center, EigenVector);
}

float COBBPCA::computeOBBVolume() const
{
	return (m_OBB._Max[0] - m_OBB._Min[0]) * (m_OBB._Max[1] - m_OBB._Min[1]) * (m_OBB._Max[2] - m_OBB._Min[2]);
}
