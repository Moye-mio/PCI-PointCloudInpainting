#include "pch.h"

#include "NurbsFitting.h"

using namespace core;

bool CNurbsFitting::run(const PC_t::Ptr& vCloud, int vDegree, int vRefinement, int vIters)
{
	_HIVE_EARLY_RETURN(vCloud == nullptr, "Cloud is nullptr", false);
	_HIVE_EARLY_RETURN(vCloud->size() == 0, "Cloud is empty", false);
	_HIVE_EARLY_RETURN(vDegree < 2, _FORMAT_STR1("Degree [%1%] < 2", vDegree), false);
	_HIVE_EARLY_RETURN(vRefinement < 1, _FORMAT_STR1("Refinement [%1%] < 1", vRefinement), false);
	_HIVE_EARLY_RETURN(vIters < 1, _FORMAT_STR1("Iterations [%1%] < 1", vIters), false);

	pcl::on_nurbs::NurbsDataSurface Data;
	__PCLPoint2ONVector3d(vCloud, Data.interior);

	int Order = vDegree + 1;
	pcl::on_nurbs::FittingSurface::Parameter Params;
	Params.interior_smoothness = 1.0;
	Params.interior_weight = 1.0;
	Params.boundary_smoothness = 1.0;
	Params.boundary_weight = 0.0;

	hiveEventLogger::hiveOutputEvent("Start Fitting Surface");
	ON_NurbsSurface Nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(Order, &Data);
	hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Nurbs Initial Control Point Number: [%1%, %2%]", Nurbs.m_cv_count[0], Nurbs.m_cv_count[1]));

	pcl::on_nurbs::FittingSurface Fit(&Data, Nurbs);

	for (int i = 0; i < vRefinement; i++)
	{
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("Refine [%1%]", i));
		Fit.refine(0);
		Fit.refine(1);
		Fit.assemble(Params);
		Fit.solve();
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Nurbs Control Point Number: [%1%, %2%]", Fit.m_nurbs.m_cv_count[0], Fit.m_nurbs.m_cv_count[1]));
	}

	for (int i = 0; i < vIters; i++)
	{
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("Iteration [%1%]", i));
		Fit.assemble(Params);
		Fit.solve();
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Nurbs Control Point Number: [%1%, %2%]", Fit.m_nurbs.m_cv_count[0], Fit.m_nurbs.m_cv_count[1]));
	}

	m_Fit = std::make_shared<pcl::on_nurbs::FittingSurface>(Fit);

	return true;
}

bool CNurbsFitting::__PCLPoint2ONVector3d(const PC_t::Ptr& vCloud, pcl::on_nurbs::vector_vec3d& voData)
{
	_HIVE_EARLY_RETURN(vCloud == nullptr, "Cloud is nullptr", false);
	_HIVE_EARLY_RETURN(vCloud->size() == 0, "Cloud is empty", false);

	if (voData.size() > 0) voData.clear();

	for (unsigned i = 0; i < vCloud->size(); i++)
	{
		const Point_t& p = vCloud->at(i);
		_HIVE_EARLY_RETURN(std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z), _FORMAT_STR4("Point [%1%] is nan, (%2%, %3%, %4%)", i, p.x, p.y, p.z), false);
		voData.push_back(Eigen::Vector3d(p.x, p.y, p.z));
	}
}

