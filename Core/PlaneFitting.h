#pragma once

namespace common
{
	struct SPlane;
}

namespace core
{
	class CPlaneFitting
	{
	public:
		CPlaneFitting() {}
		~CPlaneFitting() {}

		common::SPlane fitRansacPlane(const PC_t::Ptr& vCloud, float vDistThres);

	private:
	};
}