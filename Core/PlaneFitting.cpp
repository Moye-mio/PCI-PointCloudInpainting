#include "pch.h"

#include "PlaneFitting.h"
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "Plane.h"

#include <pcl/io/ply_io.h>

using namespace core;

common::SPlane CPlaneFitting::fitRansacPlane(const PC_t::Ptr& vCloud, float vDistThres)
{
	_ASSERTE(vDistThres > 0);
	_ASSERTE(vCloud != NULL);
	_ASSERTE(vCloud->size() >= 3);

	Eigen::VectorXf Coef;
	pcl::SampleConsensusModelPlane<Point_t>::Ptr ModelPlane(new pcl::SampleConsensusModelPlane<Point_t>(vCloud));
	pcl::RandomSampleConsensus<Point_t> Ransac(ModelPlane);
	Ransac.setDistanceThreshold(vDistThres);
	Ransac.computeModel();
	Ransac.getModelCoefficients(Coef);
	//std::cout << "Function of plane£º\n" << Coef[0] << "x + " << Coef[1] << "y + " << Coef[2] << "z + " << Coef[3] << " = 0" << std::endl;

	if (Coef[0] == 0 && Coef[1] == 0 && Coef[2] == 0)
	{
		std::cout << "Ransac failed...\tPoint Cloud Size: " << vCloud->size() << std::endl;
		Coef[0] = 1;
	}

	_ASSERTE(!(Coef[0] == 0 && Coef[1] == 0 && Coef[2] == 0));
	common::SPlane Plane(Coef[0], Coef[1], Coef[2], Coef[3]);
	Plane.normalize();

	return Plane;
}

