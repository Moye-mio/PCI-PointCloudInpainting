#include "pch.h"
#include "MultiLayerBSplineSurface.h"
#include "Triangle.h"

using namespace core;

float CMultiLayerBSplineSurface::calcProj(const SPoint& vPoint, Eigen::Vector2f& vUV)
{
	_ASSERTE(std::isnan(vPoint.x() + vPoint.y() + vPoint.z()));
	
	if (!__isMultiLayerReady())
		__generateMultiLayerNodes();

	for (int Layer = 0; Layer < m_Layers; Layer++)
	{
		if (Layer == 0)
		{
			for (int i = 0; i < m_ControlPoints.rows() - 1; i++)
				for (int k = 0; k < m_ControlPoints.cols() - 1; k++)
				{
					STriangle Tri1(m_ControlPoints(i, k), m_ControlPoints(i + 1, k), m_ControlPoints(i, k + 1));
					STriangle Tri2(m_ControlPoints(i + 1, k + 1), m_ControlPoints(i + 1, k), m_ControlPoints(i, k + 1));
					
				}

		}
	}
}

void CMultiLayerBSplineSurface::__generateMultiLayerNodes()
{
	int RowRaw = m_ControlPoints.rows();
	int ColRaw = m_ControlPoints.cols();

	for (int Layer = 0; Layer < m_Layers; Layer++)
	{
		if (Layer == 0) continue;

		float Step = 1.0f / (float)m_Sub;
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
