#pragma once

namespace core
{
	class CBorderExtractor
	{
	public:
		CBorderExtractor(const PC_t::Ptr& vCloud);
		~CBorderExtractor() = default;

		void Compute(float vRadius);
		void dumpBorderIndices(std::vector<int>& voIndices) const { voIndices = m_BorderIndices; }

	private:

	private:
		PC_t::Ptr m_pCloud;
		std::vector<int> m_BorderIndices;
	};
}