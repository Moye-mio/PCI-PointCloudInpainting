#pragma once

#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>

namespace core
{
	class CNurbsFitting
	{
	public:
		CNurbsFitting() = default;
		~CNurbsFitting() = default;

		bool run(const PC_t::Ptr& vCloud, int vDegree, int vRefinement, int vIters);
		void dumpFittingSurface(ON_NurbsSurface& voNurbs) { voNurbs = m_Fit->m_nurbs; }
		void dumpFitting(std::shared_ptr<pcl::on_nurbs::FittingSurface>& voFit) { voFit = m_Fit; }

	private:
		bool __PCLPoint2ONVector3d(const PC_t::Ptr& vCloud, pcl::on_nurbs::vector_vec3d& voData);

	private:
		std::shared_ptr<pcl::on_nurbs::FittingSurface> m_Fit;
	};
}