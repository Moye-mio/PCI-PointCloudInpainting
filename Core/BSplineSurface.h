#pragma once

namespace core
{
	using SPoint = Eigen::Vector3f;

	class CBSplineSurface
	{
	public:
		CBSplineSurface(int vDegree, bool vIsClamped = true);
		~CBSplineSurface() {}

		bool setControlPoints(const Eigen::Matrix<SPoint, -1, -1>& vPoints);
		SPoint sample(float vU, float vV);

	protected:
		SPoint __sample(const Eigen::Matrix<SPoint, -1, -1>& vPoints, float vU, float vV);

	protected:
		Eigen::Matrix<SPoint, -1, -1> m_ControlPoints;

		int m_Degree;
		int m_IsClamped;

	};

}