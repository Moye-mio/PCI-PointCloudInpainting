#include "pch.h"
#include "MultiLayerBSplineSurface.h"

using namespace core;

CMultiLayerBSplineSurface::CMultiLayerBSplineSurface(int vDegree, bool vIsClamped /*= true*/)
	: IBSplineSurface(vDegree)
	, m_PreLayers(2)
	, m_Sub(5)
	, m_MaxSub(5)
{}

bool CMultiLayerBSplineSurface::setLayer(int vLayer)
{
	_ASSERTE(vLayer >= 1);
	m_PreLayers = vLayer;
	return true;
}

bool CMultiLayerBSplineSurface::setMaxSub(int vMaxSub)
{
	_ASSERTE(vMaxSub >= 5);
	m_MaxSub = vMaxSub;
	return true;
}

float CMultiLayerBSplineSurface::calcProj(const SPoint & vPoint, Eigen::Vector2f& voUV)
{
	_ASSERTE(!std::isnan(vPoint.x() + vPoint.y() + vPoint.z()));
	_ASSERTE(m_PreLayers >= 1);
	
	if (!__isMultiLayerReady())
		__generatePreMultiLayerNodes();

	/*for (int Layer = 0; Layer < m_PreLayers; Layer++)
	{
		CTriangle HitTri;
		bool IsHit = false;
		std::optional<float> r = __isHitNodes(m_PreComputeNodes[Layer], vPoint, HitTri);
		if (r.has_value())
			__calcUV(HitTri, vPoint, vUV);
	}*/

	/*CTriangle HitTri;
	if (auto r = __HitNodes(m_PreComputeNodes[m_PreLayers - 1], vPoint, HitTri); r.has_value())
		__calcUV(m_PreComputeNodes[m_PreLayers - 1], m_PreComputeUV, HitTri, vPoint, voUV);
	float Epsilon = 0.00001f;
	if (__calcNodesDiff(vPoint, __sample(m_PreComputeNodes[m_PreLayers - 1], voUV[0], voUV[1])) >= Epsilon)
	{
		Eigen::Matrix<SPoint, -1, -1> Local, Sub;
		Eigen::Matrix<Eigen::Vector2f, -1, -1> SubUV;
		__extractLocalNodes(HitTri, m_PreComputeNodes[m_PreLayers - 1], Local);
		__subdivide(Local, Sub, SubUV);
		if (auto r = __HitNodes(Sub, vPoint, HitTri); r.has_value())
			__calcUV(Sub, SubUV, HitTri, vPoint, voUV);
	}*/

	/*CTriangle HitTri;
	float Epsilon = 0.00001f;
	Eigen::Matrix<SPoint, -1, -1> CurNodes, LastNodes, TempNodes;
	Eigen::Matrix<Eigen::Vector2f, -1, -1> CurUV;
	std::vector<Eigen::Vector2f> EdgeUV;
	for (int i = 0; i < m_MaxSub; i++)
	{
		LastNodes = CurNodes;
		if (i == 0)
		{
			CurNodes = m_PreComputeNodes[m_PreLayers - 1];
			CurUV = m_PreComputeUV;
		}
		else
		{
			__extractLocalNodes(HitTri, LastNodes, TempNodes);
			__subdivide(TempNodes, CurNodes, CurUV);
		}

		if (auto r = __HitNodes(CurNodes, vPoint, HitTri); r.has_value())
			__calcUV(CurNodes, CurUV, HitTri, vPoint, voUV);

		if (__calcNodesDiff(vPoint, __sample(CurNodes, voUV[0], voUV[1])) < Epsilon)
			break;
	}*/

	std::vector<Eigen::Vector2i> Hit;
	float Epsilon = 0.00001f;
	Eigen::Matrix<SPoint, -1, -1> CurNodes, LastNodes, TempNodes;
	Eigen::Matrix<Eigen::Vector2f, -1, -1> CurUV;
	for (int i = 0; i < m_MaxSub; i++)
	{
		LastNodes = CurNodes;
		if (i == 0)
		{
			CurNodes = m_PreComputeNodes[m_PreLayers - 1];
			CurUV = m_PreComputeUV;
		}
		else
		{
			__extractLocalNodes(LastNodes, Hit, TempNodes);
			__subdivide(TempNodes, CurNodes, CurUV);
		}
		if (auto r = __HitNodes(CurNodes, vPoint, Hit); r.has_value())
			if (auto b = __calcBaryCoor(CurNodes, Hit, vPoint); b.has_value())
			{
				Eigen::Vector3f Bary = b.value();
				std::vector<Eigen::Vector2f> TriUV;
				TriUV.emplace_back(CurUV.coeff(Hit[0][0], Hit[0][1]));
				TriUV.emplace_back(CurUV.coeff(Hit[1][0], Hit[1][1]));
				TriUV.emplace_back(CurUV.coeff(Hit[2][0], Hit[2][1]));
				voUV = __calcUV(TriUV, Bary);
			}
	}

	return 1.0f;

	//SPoint TruePoint = __sample(, voUV[0], voUV[1]);
	//return (TruePoint - vPoint).norm();
}

std::optional<core::SPoint> CMultiLayerBSplineSurface::getDetailedNode(int vRow, int vCol)
{
	const auto& CurNodes = m_PreComputeNodes[m_PreLayers - 1];
	if (vRow >= 0 && vRow < CurNodes.rows() && vCol >= 0 && vCol <= CurNodes.cols())
		return std::nullopt;
	else
		return CurNodes.coeffRef(vRow, vCol);
}

void CMultiLayerBSplineSurface::__generatePreMultiLayerNodes()
{
	int RowRaw = m_ControlPoints.rows();
	int ColRaw = m_ControlPoints.cols();

	for (int Layer = 0; Layer < m_PreLayers; Layer++)
	{
		int LastRows = (Layer == 0) ? RowRaw : m_PreComputeNodes[Layer - 1].rows();
		int LastCols = (Layer == 0) ? ColRaw : m_PreComputeNodes[Layer - 1].cols();
		int CurRows = (LastRows - 1) * m_Sub + 1;
		int CurCols = (LastCols - 1) * m_Sub + 1;
		Eigen::Matrix<SPoint, -1, -1> CurLayer;
		CurLayer.resize(CurRows, CurCols);
		if (Layer == m_PreLayers - 1)
			m_PreComputeUV.resize(CurRows, CurCols);
		for (int i = 0; i < CurRows; i++)
			for (int k = 0; k < CurCols; k++)
			{
				if (Layer == 0)
					CurLayer.coeffRef(i, k) = __sample(m_ControlPoints, (float)i / (float)(CurRows - 1), (float)k / (float)(CurCols - 1));
				else 
					CurLayer.coeffRef(i, k) = __sample(m_PreComputeNodes[Layer - 1], (float)i / (float)(CurRows - 1), (float)k / (float)(CurCols - 1));
				if (Layer == m_PreLayers - 1)
					m_PreComputeUV.coeffRef(i, k) = Eigen::Vector2f((float)i / (float)(CurRows - 1), (float)k / (float)(CurCols - 1));
			}
		m_PreComputeNodes.emplace_back(CurLayer);
	}
}

bool CMultiLayerBSplineSurface::__isMultiLayerReady()
{
	if (m_PreComputeNodes.size() != m_PreLayers) return false;
	else return true;
}

std::optional<float> CMultiLayerBSplineSurface::__HitNodes(const Eigen::Matrix<SPoint, -1, -1>& vNodes, const SPoint& vPoint, std::vector<Eigen::Vector2i>& voHit)
{
	std::vector<std::pair<float, std::vector<Eigen::Vector2i>>> Candidates;
	for (int i = 0; i < vNodes.rows() - 1; i++)
		for (int k = 0; k < vNodes.cols() - 1; k++)
		{
			CTriangle Tri1(vNodes(i, k), vNodes(i + 1, k), vNodes(i, k + 1));
			if (auto r = __HitTriangle(Tri1, vPoint); r.has_value())
			{
				std::vector<Eigen::Vector2i> Indices;
				Indices.emplace_back(Eigen::Vector2i(i, k));
				Indices.emplace_back(Eigen::Vector2i(i + 1, k));
				Indices.emplace_back(Eigen::Vector2i(i, k + 1));
				Candidates.emplace_back(std::make_pair(r.value(), Indices));
			}

			CTriangle Tri2(vNodes(i + 1, k + 1), vNodes(i + 1, k), vNodes(i, k + 1));
			if (auto r = __HitTriangle(Tri2, vPoint); r.has_value())
			{
				std::vector<Eigen::Vector2i> Indices;
				Indices.emplace_back(Eigen::Vector2i(i + 1, k + 1));
				Indices.emplace_back(Eigen::Vector2i(i + 1, k));
				Indices.emplace_back(Eigen::Vector2i(i, k + 1));
				Candidates.emplace_back(std::make_pair(r.value(), Indices));
			}
		}

	if (Candidates.empty())
		return std::nullopt;

	std::sort(Candidates.begin(), Candidates.end(),
		[&](const std::pair<float, std::vector<Eigen::Vector2i>>& a, const std::pair<float, std::vector<Eigen::Vector2i>>& b) -> bool
		{
			return a.first < b.first;
		});

	voHit = Candidates.begin()->second;
	return Candidates.begin()->first;
}

std::optional<float> CMultiLayerBSplineSurface::__HitTriangle(const CTriangle& vTri, const SPoint& vPoint)
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

Eigen::Vector2f CMultiLayerBSplineSurface::__calcUV(const std::vector<Eigen::Vector2f>& vUV, const Eigen::Vector3f& vBary)
{
	return vBary[0] * vUV[0] + vBary[1] * vUV[1] + vBary[2] * vUV[2];
}

std::optional<Eigen::Vector2i> CMultiLayerBSplineSurface::__findNodes(const Eigen::Matrix<SPoint, -1, -1>& vNodes, const SPoint& vPoint)
{
	_ASSERTE(__isMultiLayerReady());

	for (int i = 0; i < vNodes.rows(); i++)
		for (int k = 0; k < vNodes.cols(); k++)
			if (vPoint == vNodes.coeff(i, k))
				return Eigen::Vector2i(i, k);

	return std::nullopt;
}

float CMultiLayerBSplineSurface::__calcNodesDiff(const SPoint& vLhs, const SPoint& vRhs)
{
	_ASSERTE(vLhs.isValid() && vRhs.isValid());

	return (vLhs - vRhs).norm();
}

void CMultiLayerBSplineSurface::__subdivide(const Eigen::Matrix<SPoint, -1, -1>& vRough, Eigen::Matrix<SPoint, -1, -1>& voSub, Eigen::Matrix<Eigen::Vector2f, -1, -1>& voUV)
{
	int CurRows = (vRough.rows() - 1) * m_Sub + 1;
	int CurCols = (vRough.cols() - 1) * m_Sub + 1;
	voSub.resize(CurRows, CurCols);
	voUV.resize(CurRows, CurCols);
	for (int i = 0; i < CurRows; i++)
		for (int k = 0; k < CurCols; k++)
		{
			voSub.coeffRef(i, k) = __sample(vRough, (float)i / (float)(CurRows - 1), (float)k / (float)(CurCols - 1));
			voUV.coeffRef(i, k) = Eigen::Vector2f((float)i / (float)(CurRows - 1), (float)k / (float)(CurCols - 1));
		}
}

void CMultiLayerBSplineSurface::__extractLocalNodes(const Eigen::Matrix<SPoint, -1, -1>& vNodes, const std::vector<Eigen::Vector2i>& vHit, Eigen::Matrix<SPoint, -1, -1>& vLocal)
{
	Eigen::Vector2i Min = { INT_MAX , INT_MAX };
	Eigen::Vector2i Max = { -INT_MAX , -INT_MAX };
	for (const auto& e : vHit)
	{
		Min.x() = (Min.x() < e[0]) ? Min.x() : e[0];
		Min.y() = (Min.y() < e[1]) ? Min.y() : e[1];
		Max.x() = (Max.x() > e[0]) ? Max.x() : e[0];
		Max.y() = (Max.y() > e[1]) ? Max.y() : e[1];
	}

	Eigen::Vector2i Start = { Min.x() - 1, Min.y() - 1 };
	_ASSERTE(Start.x() >= 0 && Start.y() >= 0 && Start.x() < vNodes.rows() && Start.y() < vNodes.cols());
	if (Start.x() < 0) Start.x()++;
	if (Start.y() < 0) Start.y()++;
	if (Start.x() > vNodes.rows() - 4) Start.x()--;
	if (Start.y() > vNodes.cols() - 4) Start.y()--;

	vLocal.resize(4, 4);
	for (int i = 0; i < vLocal.rows(); i++)
		for (int k = 0; k < vLocal.cols(); k++)
			vLocal.coeffRef(i, k) = vNodes.coeff(Start.x() + i, Start.x() + k);
}

std::optional<Eigen::Vector3f> CMultiLayerBSplineSurface::__calcBaryCoor(const Eigen::Matrix<SPoint, -1, -1>& vNodes, const std::vector<Eigen::Vector2i>& vHit, const SPoint& vPoint)
{
	for (int i = 0; i < vHit.size(); i++)
	{
		const auto& Coor = vHit[i];
		_ASSERTE(Coor.x() >= 0 && Coor.x() < vNodes.rows() && Coor.y() >= 0 && Coor.y() < vNodes.cols());
	}

	Eigen::Vector3f Bary;
	float Epsilon = 0.00001f;
	CTriangle Tri(vNodes(vHit[0][0], vHit[0][1]), vNodes(vHit[1][0], vHit[1][1]), vNodes(vHit[2][0], vHit[2][1]));
	Tri.calcBaryCoor(vPoint, Bary);
	if (Bary[0] + Bary[1] + Bary[2] > 1.0f + Epsilon)
		return std::nullopt;
	else
		return Bary;
}

