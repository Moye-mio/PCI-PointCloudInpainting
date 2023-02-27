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
		bool compute(float vRadius);
		bool compute(int vK);
		void dumpNormals(NormalPC_t::Ptr& voNormals) { voNormals = m_Normals; }

	private:
		bool __validateNormal();
		bool __normalize(Normal_t& vioNormal);

	private:
		PC_t::Ptr			m_Cloud;
		NormalPC_t::Ptr		m_Normals;
	};
}