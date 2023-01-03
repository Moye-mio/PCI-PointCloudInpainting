#include "pch.h"
#include "MultiLayerBSplineSurface.h"

using namespace core;

bool CMultiLayerBSplineSurface::setLayer(int vLayer)
{
	_ASSERTE(vLayer >= 1);
	m_Layers = vLayer;
	return true;
}

float CMultiLayerBSplineSurface::calcProj(const SPoint & vPoint, Eigen::Vector2f& voUV)
{
	_ASSERTE(std::isnan(vPoint.x() + vPoint.y() + vPoint.z()));
	
	if (!__isMultiLayerReady())
		__generateMultiLayerNodes();

	/*for (int Layer = 0; Layer < m_Layers; Layer++)
	{
		CTriangle HitTri;
		bool IsHit = false;
		std::optional<float> r = __isHitNodes(m_MultiLayerNodes[Layer], vPoint, HitTri);
		if (r.has_value())
			__calcUV(HitTri, vPoint, vUV);
	}*/

	CTriangle HitTri;
	bool IsHit = false;
	std::optional<float> r = __isHitNodes(m_MultiLayerNodes[m_Layers - 1], vPoint, HitTri);
	if (r.has_value())
		__calcUV(HitTri, vPoint, voUV);

	SPoint TruePoint = __sample(m_MultiLayerNodes[m_Layers - 1], voUV[0], voUV[1]);
	return (TruePoint - vPoint).norm();
}

void CMultiLayerBSplineSurface::__generateMultiLayerNodes()
{
	int RowRaw = m_ControlPoints.rows();
	int ColRaw = m_ControlPoints.cols();

	for (int Layer = 0; Layer < m_Layers; Layer++)
	{
		int LastRows = (Layer == 0) ? RowRaw : m_MultiLayerNodes[Layer - 1].rows();
		int LastCols = (Layer == 0) ? ColRaw : m_MultiLayerNodes[Layer - 1].cols();
		int CurRows = (LastRows - 1) * m_Sub + 1;
		int CurCols = (LastCols - 1) * m_Sub + 1;
		Eigen::Matrix<SPoint, -1, -1> CurLayer;
		CurLayer.resize(CurRows, CurCols);
		for (int i = 0; i < CurRows; i++)
			for (int k = 0; k < CurCols; k++)
			{
				if (Layer == 0)
					CurLayer.coeffRef(i, k) = __sample(m_ControlPoints, (float)i / (float)(CurRows - 1), (float)k / (float)(CurCols - 1));
				else 
					CurLayer.coeffRef(i, k) = __sample(m_MultiLayerNodes[Layer - 1], (float)i / (float)(CurRows - 1), (float)k / (float)(CurCols - 1));
				if (Layer == m_Layers - 1)
					m_LatestLayerUV.coeffRef(i, k) = Eigen::Vector2f((float)i / (float)(CurRows - 1), (float)k / (float)(CurCols - 1));
			}
		m_MultiLayerNodes.emplace_back(CurLayer);
	}
}

bool CMultiLayerBSplineSurface::__isMultiLayerReady()
{
	if (m_MultiLayerNodes.size() != m_Layers) return false;
	else return true;
}

std::optional<float> CMultiLayerBSplineSurface::__isHitNodes(const Eigen::Matrix<SPoint, -1, -1>& vNodes, const SPoint& vPoint, CTriangle& voTri)
{
	std::vector<std::pair<float, CTriangle>> Candidates;
	for (int i = 0; i < vNodes.rows() - 1; i++)
		for (int k = 0; k < vNodes.cols() - 1; k++)
		{
			CTriangle Tri1(vNodes(i, k), vNodes(i + 1, k), vNodes(i, k + 1));
			if (auto r = __isHitTriangle(Tri1, vPoint); r.has_value())
				Candidates.emplace_back(std::make_pair(r.value(), Tri1));

			CTriangle Tri2(vNodes(i + 1, k + 1), vNodes(i + 1, k), vNodes(i, k + 1));
			if (auto r = __isHitTriangle(Tri2, vPoint); r.has_value())
				Candidates.emplace_back(std::make_pair(r.value(), Tri2));
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

bool CMultiLayerBSplineSurface::__calcUV(const CTriangle& vTri, const SPoint& vPoint, Eigen::Vector2f& voUV)
{
	Eigen::Vector3f Coor;
	vTri.calcBaryCoor(vPoint, Coor);
	_ASSERTE(!std::isnan(Coor.norm()));

	std::vector<Eigen::Vector2f> UVs;
	for (int i = 0; i < vTri.size(); i++)
	{
		if (auto r = __findUV(vTri[i]); r.has_value())
			UVs.emplace_back(std::move(r.value()));
		else
			return false;
	}

	voUV = Coor[0] * UVs[0] + Coor[1] * UVs[1] + Coor[2] * UVs[2];
	return true;
}

std::optional<Eigen::Vector2f> CMultiLayerBSplineSurface::__findUV(const SPoint& vPoint)
{
	_ASSERTE(__isMultiLayerReady());

	const auto& Nodes = m_MultiLayerNodes[m_Layers - 1];
	for (int i = 0; i < Nodes.rows(); i++)
		for (int k = 0; k < Nodes.cols(); k++)
			if (vPoint == Nodes.coeff(i, k))
				return m_LatestLayerUV.coeff(i, k);

	return std::nullopt;
}


