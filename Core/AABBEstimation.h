#pragma once

namespace core
{
	struct SAABB;

	class CAABBEstimation
	{
	public:
		CAABBEstimation(const PC_t::Ptr& vCloud);
		~CAABBEstimation() = default;

		SAABB compute();

	private:
		PC_t::Ptr m_pCloud;

	};
}