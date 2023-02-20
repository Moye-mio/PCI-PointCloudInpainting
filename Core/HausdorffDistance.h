#pragma once

namespace core
{
	class CHausDorffDistance
	{
	public:
		CHausDorffDistance() = default;
		~CHausDorffDistance() = default;

		std::optional<float> compute(const PC_t::Ptr& vLhs, const PC_t::Ptr& vRhs);

	private:

	};
}