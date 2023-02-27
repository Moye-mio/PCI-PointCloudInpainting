#include "pch.h"
#include "NormalWrapper.h"
#include "NormalEstimator.h"

using namespace core;

bool CNormalWrapper::compute(const std::vector<SVertex>& vVertices, int vK, std::vector<Eigen::Vector3f>& voNormals)
{
	_HIVE_EARLY_RETURN(vVertices.size() == 0, "Normal Wrapper: Vertices are empty", false);
	_HIVE_EARLY_RETURN(vK < 0, "Normal Wrapper: Radius < 0", false);

	PC_t::Ptr pCloud(new PC_t);
	{
		int Number = 0;
		for (const auto& e : vVertices)
		{
			_HIVE_EARLY_RETURN(e.isValid() == false, _FORMAT_STR4("Normal Wrapper: Vertex [%1%] is not valid, Value: %2%, %3%, %4%", Number, e.x, e.y, e.z), false);
			pCloud->emplace_back(Point_t(e.x, e.y, e.z));
			Number++;
		}
	}

	NormalPC_t::Ptr pNormals(new NormalPC_t);
	core::CNormalEstimator Estimator;
	_HIVE_EARLY_RETURN(Estimator.setCloud(pCloud) == false, "Normal Wrapper: Estimator set cloud failed", false);
	Estimator.compute(vK);
	Estimator.dumpNormals(pNormals);
	_HIVE_EARLY_RETURN(pNormals->size() != pCloud->size(), "Normal Wrapper: Unexpected Error, Normal size != Cloud size", false);

	_HIVE_EARLY_RETURN(__transPCLNormal2Normal(pNormals, voNormals) == false, "Normal Wrapper: PCL Normals are not Valid", false);
	return true;
}

bool CNormalWrapper::__transVertex2PCLPoint(const std::vector<SVertex>& vVertices, PC_t::Ptr& voCloud)
{
	if (voCloud == NULL) voCloud = std::make_shared<PC_t>();
	voCloud->clear();

	int Number = 0;
	for (const auto& e : vVertices)
	{
		_HIVE_EARLY_RETURN(e.isValid() == false, _FORMAT_STR4("Normal Wrapper: Vertex [%1%] is not valid, Value: %2%, %3%, %4%", Number, e.x, e.y, e.z), false);
		voCloud->emplace_back(Point_t(e.x, e.y, e.z));
		Number++;
	}

	_HIVE_EARLY_RETURN(voCloud->size() != vVertices.size(), "Unexpected ERROR: Cloud size != Vertices size", false);
}

bool CNormalWrapper::__transPCLNormal2Normal(const NormalPC_t::Ptr& vNormals, std::vector<Eigen::Vector3f>& voNormals)
{
	int Number = 0;
	for (const auto& e : *vNormals)
	{
		_HIVE_EARLY_RETURN(std::isnan(e.normal_x) || std::isnan(e.normal_y) || std::isnan(e.normal_z), _FORMAT_STR4("Normal Wrapper: PCL Normal [%1%] is nor valid, Value: %2%, %3%, %4%", Number, e.normal_x, e.normal_y, e.normal_z), false);
		voNormals.emplace_back(Eigen::Vector3f(e.normal_x, e.normal_y, e.normal_z));
		Number++;
	}
	return true;
}

