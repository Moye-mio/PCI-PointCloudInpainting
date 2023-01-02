#include "pch.h"
#include "MultiLayerBSplineSurface.h"

using namespace core;

float CMultiLayerBSplineSurface::calcProj(const SPoint& vPoint, Eigen::Vector2f& vUV)
{
	_ASSERTE(std::isnan(vPoint.x() + vPoint.y() + vPoint.z()));
	
	if (!__isMultiLayerReady())
		__generateMultiLayerNodes();

	for (int Layer = 0; Layer < m_Layers; Layer++)
	{
		CTriangle HitTri;
		bool IsHit = false;
		if (Layer == 0)
			auto r = __isHitNodes(m_ControlPoints, vPoint, HitTri);
		else
			auto r = __isHitNodes(m_MultiLayerNodes[Layer - 1], vPoint, HitTri);


	}
	return 1.0f;
}

void CMultiLayerBSplineSurface::__generateMultiLayerNodes()
{
	int RowRaw = m_ControlPoints.rows();
	int ColRaw = m_ControlPoints.cols();

	for (int Layer = 0; Layer < m_Layers; Layer++)
	{
		if (Layer == 0) continue;

		int LastRows = (Layer == 1) ? RowRaw : m_MultiLayerNodes[Layer - 1].rows();
		int LastCols = (Layer == 1) ? ColRaw : m_MultiLayerNodes[Layer - 1].cols();
		int CurRows = (LastRows - 1) * m_Sub + 1;
		int CurCols = (LastCols - 1) * m_Sub + 1;
		Eigen::Matrix<SPoint, -1, -1> CurLayer;
		CurLayer.resize(CurRows, CurCols);
		for (int i = 0; i < CurRows; i++)
			for (int k = 0; k < CurCols; k++)
			{
				if (Layer == 1) 
					CurLayer.coeffRef(i, k) = __sample(m_ControlPoints, (float)i / (float)(CurRows - 1), (float)k / (float)(CurCols - 1));
				else 
					CurLayer.coeffRef(i, k) = __sample(m_MultiLayerNodes[Layer - 1], (float)i / (float)(CurRows - 1), (float)k / (float)(CurCols - 1));
			}
		m_MultiLayerNodes.emplace_back(CurLayer);
	}
}

bool CMultiLayerBSplineSurface::__isMultiLayerReady()
{
	if (m_MultiLayerNodes.size() != m_Layers - 1) return false;
	else return true;
}

std::optional<float> CMultiLayerBSplineSurface::__isHitNodes(const Eigen::Matrix<SPoint, -1, -1>& vNodes, const SPoint& vPoint, CTriangle& voTri)
{
	std::unordered_map<float, CTriangle> Candidates;
	for (int i = 0; i < vNodes.rows() - 1; i++)
		for (int k = 0; k < vNodes.cols() - 1; k++)
		{
			CTriangle Tri1(vNodes(i, k), vNodes(i + 1, k), vNodes(i, k + 1));
			if (auto r = __isHitTriangle(Tri1, vPoint); r.has_value())
				Candidates.emplace(std::make_pair(r.value(), Tri1));

			CTriangle Tri2(vNodes(i + 1, k + 1), vNodes(i + 1, k), vNodes(i, k + 1));
			if (auto r = __isHitTriangle(Tri2, vPoint); r.has_value())
				Candidates.emplace(std::make_pair(r.value(), Tri2));
		}

	if (Candidates.empty())
		return std::nullopt;

	std::sort(Candidates.begin(), Candidates.end(),
		[&](const std::pair<float, CTriangle>& a, const std::pair<float, CTriangle>& b) -> bool
		{
			return a.first < b.first;
		});

	voTri = Candidates.begin()->second;
	return Candidates.begin()->first;
}

std::optional<float> CMultiLayerBSplineSurface::__isHitTriangle(const CTriangle& vTri, const SPoint& vPoint)
{
	common::SPlane Plane;
	vTri.calcPlane(Plane);
	Eigen::Vector3f ProjRay;
	float Dist = Plane.calcPointProject(vPoint, ProjRay);
	_ASSERTE(!std::isnan(Dist));
	bool r = vTri.isRayIntersection(vPoint, ProjRay);
	if (r == false)
		return std::nullopt;
	else
		return Dist;
}

