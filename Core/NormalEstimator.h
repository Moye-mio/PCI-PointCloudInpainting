#pragma once

namespace core
{
	/* Based on PCL */
	class CNormalEstimator
	{
	public:
		CNormalEstimator();
		~CNormalEstimator() {}

		[[nodiscard]] bool setCloud(const PC_t::Ptr& vCloud);
		void compute(float vRadius);
		void dumpNormals(NormalPC_t::Ptr& voNormals) { voNormals = m_Normals; }

	private:
		PC_t::Ptr			m_Cloud;
		NormalPC_t::Ptr		m_Normals;
	};
}