#pragma once

#include "MultilayerSurface.h"

namespace dataManagement
{
	class CNurbs2CloudMapper
	{
	public:
		CNurbs2CloudMapper();
		~CNurbs2CloudMapper() = default;

		bool run(const core::CHeightMap& vMask, const core::CHeightMap& vInpainted, int vSPP, const std::shared_ptr<pcl::on_nurbs::FittingSurface>& vFit, const std::shared_ptr<core::CMultilayerSurface>& vSurface);
		void dumpCloud(PC_t::Ptr& voCloud) { voCloud = m_pCloud; }

	private:
		PC_t::Ptr m_pCloud;

	};
}