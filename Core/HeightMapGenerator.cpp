#include "pch.h"

#include "HeightMapGenerator.h"
#include "AABBEstimation.h"
#include "BSplineSurface.h"
#include "Voxelization.h"


using namespace core;

CHeightMapGenerator::CHeightMapGenerator()
	: m_pCloud(new PC_t)
{ }

bool CHeightMapGenerator::setCloud(const PC_t::Ptr& vCloud)
{
	_ASSERTE(vCloud != nullptr);
	_ASSERTE(vCloud->size());
	m_pCloud = vCloud;
	return true;
}

bool CHeightMapGenerator::setAABB(const SAABB& vBox)
{
	_ASSERTE(vBox.isValid());
	m_Box = vBox;
	return true;
}

bool CHeightMapGenerator::generate(int vWidth, int vHeight)
{
	_ASSERTE(vWidth > 0 && vHeight > 0);
	m_Map.setSize(vWidth, vHeight);

	// By default, it is projected along the Z-axis

	if (!m_Box.isValid())
	{
		CAABBEstimation AABBEstimation(m_pCloud);
		m_Box = AABBEstimation.compute();
		_ASSERTE(m_Box.isValid());
	}

	for (auto& e : *m_pCloud) 
	{
		Eigen::Vector2i Offset = __computeOffset(e);
		if (e.z > m_Map.getValueAt(Offset[0], Offset[1]))
			m_Map.setValueAt(e.z, Offset[0], Offset[1]);
	}

	return true;
}

core::SPoint __transPCLPoint2SPoint(const Point_t& vP)
{
	return core::SPoint(Eigen::Vector3f(vP.x, vP.y, vP.z));
}

bool CHeightMapGenerator::generateBySurface(const std::shared_ptr<core::CMultiLayerBSplineSurface>& vSurface, int vWidth, int vHeight)
{
	_ASSERTE(vWidth > 0 && vHeight > 0);
	_ASSERTE(m_pCloud->size());
	m_Map.setSize(vWidth, vHeight);

	for (const auto& e : *m_pCloud)
	{
		const core::SPoint p = __transPCLPoint2SPoint(e);
		Eigen::Vector2f UV;
		float Dist = vSurface->calcProj(p, UV);
		Eigen::Vector2i Offset = __computeOffset(UV);

		if (Dist > m_Map.getValueAt(Offset[0], Offset[1]))
			m_Map.setValueAt(Dist, Offset[0], Offset[1]);
	}

	return true;
}


Eigen::Vector2i CHeightMapGenerator::__computeOffset(const Point_t& vPoint)
{
	int OffsetX = (vPoint.getVector3fMap()[0] - m_Box._Min[0]) / (m_Box._Max[0] - m_Box._Min[0]) * m_Map.getWidth();
	int OffsetY = (vPoint.getVector3fMap()[1] - m_Box._Min[1]) / (m_Box._Max[1] - m_Box._Min[1]) * m_Map.getHeight();

	if (m_Box._Max[0] == m_Box._Min[0]) OffsetX = 0;
	if (m_Box._Max[1] == m_Box._Min[1]) OffsetY = 0;

	if (OffsetX == m_Map.getWidth()) OffsetX--;
	if (OffsetY == m_Map.getHeight()) OffsetY--;

	return Eigen::Vector2i(OffsetX, OffsetY);
}

Eigen::Vector2i CHeightMapGenerator::__computeOffset(const Eigen::Vector2f& vUV)
{
	_ASSERTE(m_Map.getWidth() && m_Map.getHeight());
	Eigen::Vector2f Scale(1.0f / m_Map.getWidth(), 1.0f / m_Map.getHeight());
	_ASSERTE(Scale[0] && Scale[1]);

	int OffsetX = vUV[0] / Scale[0];
	int OffsetY = vUV[1] / Scale[1];

	if (OffsetX == m_Map.getWidth()) OffsetX--;
	if (OffsetY == m_Map.getHeight()) OffsetY--;

	return Eigen::Vector2i(OffsetX, OffsetY);
}

