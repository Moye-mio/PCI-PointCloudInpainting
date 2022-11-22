#pragma once

#include "PointCloudType.h"
#include "OBB.h"

namespace core
{
	class COBBPCA
	{
	public:
		COBBPCA() {}
		~COBBPCA() {}

		void compute(const PC_t::Ptr& vCloud);
		SOBB getOBB() const { return m_OBB; }
		float computeOBBVolume() const;

	private:
		SOBB m_OBB;
	};

}