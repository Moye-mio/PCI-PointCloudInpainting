#pragma once

#include "MultilayerSurface.h"
#include "HeightMap.h"

namespace dataManagement
{
	class CSurface2CloudMapper
	{
	public:
		CSurface2CloudMapper();
		~CSurface2CloudMapper() = default;

		bool setSurface(const std::shared_ptr<core::CMultilayerSurface>& vSurface);
		bool map2Cloud(const core::CHeightMap& vRaw, const core::CHeightMap& vInpainted, int vSPP);
		void dumpCloud(PC_t::Ptr& voCloud) { voCloud = m_NewCloud; }

	private:
		bool __ValidCheck(const core::CHeightMap& vRaw, const core::CHeightMap& vInpainted, int vSPP);

	private:
		std::shared_ptr<core::CMultilayerSurface> m_pSurface;
		PC_t::Ptr m_NewCloud;
	};
}