#include "pch.h"
#include "NormalSampler.h"

using namespace core;

bool CNormalSampler::setData(const Eigen::Matrix<SVertex, -1, -1>& vData)
{
	_HIVE_EARLY_RETURN(vData.size() == 0, "ERROR: Surface Sampler set Data Size 0...", false);
	m_Vertices = vData;
	return true;
}

std::optional<Eigen::Vector3f> CNormalSampler::sample(const Eigen::Vector2f& vUV)
{
	_HIVE_EARLY_RETURN(__isUVValid(vUV) == false, "ERROR: InValid Input UV...", std::nullopt);
	_HIVE_EARLY_RETURN(m_Vertices.size() == 0, "ERROR: Data size 0...", std::nullopt);

	for (int i = 0; i < m_Vertices.rows() - 1; i++)
		for (int k = 0; k < m_Vertices.cols() - 1; k++)
		{
			float CurU = m_Vertices.coeff(i, k).u;
			float CurV = m_Vertices.coeff(i, k).v;
			float DiagU = m_Vertices.coeff(i + 1, k + 1).u;
			float DiagV = m_Vertices.coeff(i + 1, k + 1).v;
			
			if (CurU <= vUV[0] && vUV[0] <= DiagU && CurV <= vUV[1] && vUV[1] <= DiagV)
			{
				CTriangle Tri1(m_Vertices(i, k).getUV3f(), m_Vertices(i + 1, k).getUV3f(), m_Vertices(i, k + 1).getUV3f());
				CTriangle Tri2(m_Vertices(i + 1, k + 1).getUV3f(), m_Vertices(i + 1, k).getUV3f(), m_Vertices(i, k + 1).getUV3f());
				std::vector<CTriangle> Tris;
				Tris.emplace_back(Tri1);
				Tris.emplace_back(Tri2);

				Eigen::Vector3f Coor;
				common::SPlane Plane;

				int Flag = 0;
				for (const auto& e : Tris)
				{
					bool r = e.calcBaryCoor(Eigen::Vector3f(vUV[0], vUV[1], 0), Coor);
					if (r == true)
					{
						CTriangle Actual;
						if (Flag == 0)
							Actual = CTriangle(m_Vertices(i, k).getXYZ(), m_Vertices(i + 1, k).getXYZ(), m_Vertices(i, k + 1).getXYZ());
						else
							Actual = CTriangle(m_Vertices(i + 1, k + 1).getXYZ(), m_Vertices(i + 1, k).getXYZ(), m_Vertices(i, k + 1).getXYZ());

						_HIVE_EARLY_RETURN(Actual.calcPlane(Plane) == false, "ERROR: Normal Calc Failed...", std::nullopt);
						return Eigen::Vector3f(Plane._A, Plane._B, Plane._C);
					}
					Flag++;
				}

				hiveEventLogger::hiveOutputEvent("ERROR: Normal Calc no suitable triangle...");
				return std::nullopt;
			}
		}
}

bool CNormalSampler::__isUVValid(const Eigen::Vector2f& vUV)
{
	if (vUV.x() >= 0 && vUV.x() <= 1 && vUV.y() >= 0 && vUV.y() <= 1)	return true;
	else																return false;
}
