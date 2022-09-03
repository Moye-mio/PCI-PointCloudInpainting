#pragma once

namespace dataManagement
{
	// an encapsulation of PCL PointCloud
	class CPCWrapper
	{
	public:
		CPCWrapper(const PC_t::Ptr& vPC, const std::string& vId);
		~CPCWrapper();

		const PC_t::Ptr	getPointCloud() const { return m_pPC; };
		std::uint64_t getSize() const { return m_pPC->size(); };

	private:
		PC_t::Ptr m_pPC;
		std::string m_Id;

	};
}