#pragma once

namespace core
{
	class CNurbsFitting
	{
	public:
		CNurbsFitting();
		~CNurbsFitting() = default;

		bool run(int vDegree, );

	private:
		int m_Degree;
		Eigen::Matrix<SVertex, -1, -1> m_Ctrlpts;


	};
}