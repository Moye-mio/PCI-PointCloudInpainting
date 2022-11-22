#pragma once

namespace core
{
	class CPlaneFitting
	{
	public:
		CPlaneFitting();
		~CPlaneFitting() {}

		Eigen::VectorXf fitRansacPlane(const PC_t::Ptr& vCloud, float vDistThres);

	private:
		Eigen::VectorXf __correct(const Eigen::VectorXf& vCoef);

	private:
		float m_Epsilon;
	};
}