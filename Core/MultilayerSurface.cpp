#include "pch.h"

#include "MultilayerSurface.h"
#include "BSplineCurve.h"

using namespace core;

CMultilayerSurface::CMultilayerSurface(int vDegree, bool vIsClamped /*= true*/)
	: IBSplineSurface(vDegree)
	, m_SubNum(2)
	, m_SubLayer(5)
	, m_IsCalcError(true)
{}

bool CMultilayerSurface::setSubNumber(int vSubNum)
{
	if (vSubNum <= 0)
		return false;
	m_SubNum = vSubNum;
	return true;
}

bool CMultilayerSurface::setSubLayer(int vSubLayer)
{
	if (vSubLayer <= 0)
		return false;
	m_SubLayer = vSubLayer;
	return true;
}

bool CMultilayerSurface::setIsCalcError(bool vIsCalcError)
{
	m_IsCalcError = vIsCalcError;
	return true;
}

std::optional<core::SProjInfo> CMultilayerSurface::calcProj(const SPoint& vPoint)
{
	if (!vPoint.isValid())
		return std::nullopt;

	if (__IsComputed() == false)
		if (__preCompute() == false)
			return std::nullopt;
	
	SProjInfo Info;
	std::vector<Eigen::Vector2i> Hit;
	for (int i = 0; i < m_SubLayer; i++)
	{
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("Layer [%1%]", i));

		const auto& CurLayer = m_Vertices[i];
		std::pair<Eigen::Vector2i, Eigen::Vector2i> Range;
		if (i == 0)
			Range = std::make_pair(Eigen::Vector2i(0, 0), Eigen::Vector2i(CurLayer.rows() - 2, CurLayer.cols() - 2));
		else
			__calcRange(Hit, Range);

		Hit.clear();
		Hit.shrink_to_fit();

		auto r = __calcHitNodes(i, vPoint, Range, Hit);
		_HIVE_EARLY_RETURN(r.has_value() == false, "ERROR: No Hit...", std::nullopt);

		Info._Dist = r.value();
	}

	return Info;
}

bool CMultilayerSurface::__preCompute()
{
	if (m_ControlPoints.size() == 0) return false;

	for (int i = 0; i < m_SubLayer; i++)
	{
		Eigen::Matrix<SVertex, -1, -1> CurLayer;
		if (i == 0)
		{
			CurLayer.resize(m_ControlPoints.rows(), m_ControlPoints.cols());
			for (int Row = 0; Row < m_ControlPoints.rows(); Row++)
				for (int Col = 0; Col < m_ControlPoints.cols(); Col++)
				{
					const auto p = m_ControlPoints.coeff(Row, Col);
					if (auto r = __transPoint2Vertex(p, Eigen::Vector2f((float)Row / (float)m_ControlPoints.rows(), (float)Col / (float)m_ControlPoints.cols())); r.has_value())
						CurLayer.coeffRef(Row, Col) = r.value();
					else
					{
#ifdef _LOG
						std::cout << "Error: m_ControlPoints Index (" << Row << ", " << Col << ") \tValues (" << p.x() << ", " << p.y() << ", " << p.z() << ")\n";
#endif // _LOG
							return false;
					}
				}

			m_Vertices.emplace_back(std::move(CurLayer));
			continue;
		}

		const auto LastLayer = m_Vertices[i - 1];
		int LastRows = LastLayer.rows();
		int LastCols = LastLayer.cols();
		int CurRows = (LastRows - 1) * m_SubNum + 1;
		int CurCols = (LastCols - 1) * m_SubNum + 1;

		CurLayer.resize(CurRows, CurCols);
		for (int Row = 0; Row < LastLayer.rows(); Row++)
			for (int Col = 0; Col < LastLayer.cols(); Col++)
			{
				if (auto r = __sample(LastLayer, (float)Row / (float)(CurRows - 1), (float)Col / (float)(CurCols - 1)); r.has_value())
					CurLayer.coeffRef(Row, Col) = r.value();
				else
				{
#ifdef _LOG
					std::cout << "Error: Layer Sample Failed...Layer: " << i << "Index(" << Row << ", " << Col << ")" << std::endl;
#endif // _LOG
					return false;
				}
			}

		m_Vertices.emplace_back(std::move(CurLayer));
	}

#ifdef _LOG
	for (int i = 0; i < m_Vertices.size(); i++)
		std::cout << "Layer " << i << " Size: " << m_Vertices[i].size() << std::endl;
#endif // _LOG

	return true;
}

bool CMultilayerSurface::__IsComputed()
{
	if (m_Vertices.size() < m_SubLayer) return false;
	else								return true;
}

std::optional<float> CMultilayerSurface::__HitTriangle(const CTriangle& vTri, const SPoint& vPoint)
{
	common::SPlane Plane;
	vTri.calcPlane(Plane, false);
	Eigen::Vector3f ProjRay;
	float Dist = Plane.calcPointProject(vPoint, ProjRay);
	_ASSERTE(!std::isnan(Dist));
	bool r = vTri.isRayIntersection(vPoint, ProjRay);
	if (r == false)
		return std::nullopt;
	else
		return Dist;
}

std::optional<float> CMultilayerSurface::__calcPointDist(const SPoint& vLhs, const SPoint& vRhs)
{
	if (!vLhs.isValid() || !vRhs.isValid())	return std::nullopt;
	else									return (vLhs - vRhs).norm();
}

std::optional<core::SVertex> CMultilayerSurface::__transPoint2Vertex(const SPoint& vPoint, const Eigen::Vector2f vUV /*= Eigen::Vector2f(0, 0)*/) {
	if (vPoint.isValid() == false)
		return std::nullopt;

	SVertex Vertex(vPoint.x(), vPoint.y(), vPoint.z(), vUV[0], vUV[1]);
	return Vertex;
}

std::optional<core::SPoint> CMultilayerSurface::__transVertex2Point(const SVertex& vVertex)
{
	if (vVertex.isValid() == false)
		return std::nullopt;

	SPoint Point(vVertex.x, vVertex.y, vVertex.z);
	return Point;
}

std::optional<core::SVertex> CMultilayerSurface::__sample(const Eigen::Matrix<SVertex, -1, -1>& vNodes, float vU, float vV)
{
	std::vector<SPoint> NodesInRow;
	for (int i = 0; i < vNodes.cols(); i++)
	{
		std::vector<SPoint> NodesPerCol;
		for (int k = 0; k < vNodes.rows(); k++)
		{
			if (auto r = __transVertex2Point(vNodes.coeff(k, i)); r.has_value())
				NodesPerCol.emplace_back(r.value());
			else
				return std::nullopt;
		}
		core::CBSplineCurve Curve(m_Degree);
		Curve.setControlPoints(NodesPerCol);
		SPoint r = Curve.sample(vU);
		NodesInRow.emplace_back(r);
	}

	core::CBSplineCurve Curve(m_Degree);
	Curve.setControlPoints(NodesInRow);
	SPoint Result = Curve.sample(vV);

	if (auto r = __transPoint2Vertex(Result, Eigen::Vector2f(vU, vV)); r.has_value())
		return r.value();
	else
		return std::nullopt;
}

std::optional<float> CMultilayerSurface::__calcHitNodes(int vLayer, const SPoint& vPoint, const std::pair<Eigen::Vector2i, Eigen::Vector2i>& vRange, std::vector<Eigen::Vector2i>& voHit)
{
	if (vLayer < 0 || vLayer >= m_SubLayer)	return std::nullopt;
	if (!vPoint.isValid())					return std::nullopt;

	const auto& CurNodes = m_Vertices[vLayer];
	const auto& Start = vRange.first;
	const auto& End = vRange.second;
	if (Start.x() < 0 || Start.y() < 0)										return std::nullopt;
	if (Start.x() > End.x() || Start.y() > End.y())							return std::nullopt;
	if (End.x() >= CurNodes.rows() - 1 || End.y() >= CurNodes.cols() - 1)	return std::nullopt;
	
	std::vector<std::pair<float, std::vector<Eigen::Vector2i>>> Candidates;

	for (int i = Start.x(); i < End.x(); i++)
		for (int k = Start.y(); k < End.y(); k++)
		{
			std::vector<Eigen::Vector2i> Indices;
			Indices.emplace_back(Eigen::Vector2i(i, k));
			Indices.emplace_back(Eigen::Vector2i(i + 1, k));
			Indices.emplace_back(Eigen::Vector2i(i, k + 1));
			Indices.emplace_back(Eigen::Vector2i(i + 1, k + 1));

			std::vector<std::pair<CTriangle, int>> Tris;
			CTriangle Tri1, Tri2;

			if (auto r = __geneTriangle(CurNodes(i, k), CurNodes(i + 1, k), CurNodes(i, k + 1)); r.has_value())	Tri1 = r.value(); else return std::nullopt;
			if (auto r = __geneTriangle(CurNodes(i + 1, k + 1), CurNodes(i + 1, k), CurNodes(i, k + 1)); r.has_value())	Tri2 = r.value(); else return std::nullopt;
			Tris.emplace_back(std::make_pair(Tri1, 0));
			Tris.emplace_back(std::make_pair(Tri2, 1));

			for (const auto& e : Tris)
			{
				if (auto r = __HitTriangle(e.first, vPoint); r.has_value())
				{
#ifdef _LOG
					std::cout << "Hit: Triangle (" << i << ", " << k << "), (" << i + 1 << ", " << k << "), (" << i << ", " << k + 1 << ")\tDist: " << r.value() << std::endl;
#endif // _LOG
					std::vector<Eigen::Vector2i> CandIndices;
					CandIndices.emplace_back(Indices[e.second]);
					CandIndices.emplace_back(Indices[e.second + 1]);
					CandIndices.emplace_back(Indices[e.second + 2]);
					Candidates.emplace_back(std::make_pair(r.value(), CandIndices));
				}
				else
				{
#ifdef _LOG
					//std::cout << "Miss: \t";
					//std::cout << "(" << i << ", " << k << "), (" << i + 1 << ", " << k << "), (" << i << ", " << k + 1 << ")\t";
					//std::cout << "Triangle: (" << CurNodes(i, k).x << ", " << CurNodes(i, k).y << ", " << CurNodes(i, k).z << "), (" << CurNodes(i + 1, k).x << ", " << CurNodes(i + 1, k).y << ", " << CurNodes(i + 1, k).z << "), (" << CurNodes(i, k + 1).x << ", " << CurNodes(i, k + 1).y << ", " << CurNodes(i, k + 1).z << ")" << std::endl;
#endif // _LOG
				}
			}
		}

	_HIVE_EARLY_RETURN(Candidates.empty(), "ERROR: Candidates Empty...", std::nullopt);
	
	std::sort(Candidates.begin(), Candidates.end(),
		[&](const std::pair<float, std::vector<Eigen::Vector2i>>& a, const std::pair<float, std::vector<Eigen::Vector2i>>& b) -> bool
		{
			return a.first < b.first;
		});

	hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("Hit Number: [%1%]", Candidates.size()));

	voHit = Candidates.begin()->second;
	return Candidates.begin()->first;
}

std::optional<core::CTriangle> CMultilayerSurface::__geneTriangle(const SVertex& vP1, const SVertex& vP2, const SVertex& vP3)
{
	if (!vP1.isValid() || !vP2.isValid() || !vP3.isValid()) return std::nullopt;

	SPoint P1, P2, P3;
	if (auto r = __transVertex2Point(vP1); r.has_value()) P1 = r.value(); else return std::nullopt;
	if (auto r = __transVertex2Point(vP2); r.has_value()) P2 = r.value(); else return std::nullopt;
	if (auto r = __transVertex2Point(vP3); r.has_value()) P3 = r.value(); else return std::nullopt;

	CTriangle Tri(P1, P2, P3);
	return Tri;
}

void CMultilayerSurface::__calcRange(const std::vector<Eigen::Vector2i>& vHit, std::pair<Eigen::Vector2i, Eigen::Vector2i>& voRange)
{
	Eigen::Vector2i Min(INT_MAX, INT_MAX);
	for (const auto& e : vHit)
	{
		Min.x() = (Min.x() < e.x()) ? Min.x() : e.x();
		Min.y() = (Min.y() < e.y()) ? Min.y() : e.y();
	}

	Min = Min * m_SubNum;
	voRange = std::make_pair(Min, Eigen::Vector2i(Min.x() + m_SubNum - 1, Min.y() + m_SubNum - 1));
}


