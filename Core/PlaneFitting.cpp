#include "pch.h"

#include "PlaneFitting.h"
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace core;

CPlaneFitting::CPlaneFitting()
	: m_Epsilon(0.00001f)
{}

Eigen::VectorXf CPlaneFitting::fitRansacPlane(const PC_t::Ptr& vCloud, float vDistThres)
{
	_ASSERTE(vDistThres > 0);
	_ASSERTE(vCloud != NULL);
	_ASSERTE(vCloud->size());

	Eigen::VectorXf Coef;
	pcl::SampleConsensusModelPlane<Point_t>::Ptr ModelPlane(new pcl::SampleConsensusModelPlane<Point_t>(vCloud));
	pcl::RandomSampleConsensus<Point_t> Ransac(ModelPlane);
	Ransac.setDistanceThreshold(vDistThres);
	Ransac.computeModel();
	Ransac.getModelCoefficients(Coef);
	//std::cout << "Function of plane£º\n" << Coef[0] << "x + " << Coef[1] << "y + " << Coef[2] << "z + " << Coef[3] << " = 0" << std::endl;

	return __correct(Coef);
}

Eigen::VectorXf CPlaneFitting::__correct(const Eigen::VectorXf& vCoef)
{
	_ASSERTE(vCoef.size() == 4);

	Eigen::VectorXf Coefs = vCoef;
	for (int i = 0; i < vCoef.size(); i++)
	{
		if (std::fabsf(vCoef[i]) < m_Epsilon)
			Coefs[i] = 0.0f;

		if (vCoef[i] > 0)
			break;
		else if (vCoef[i] < 0)
		{
			Coefs = vCoef * -1;
			break;
		}
	}
	return Coefs;
}

