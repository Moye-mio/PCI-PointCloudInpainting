#include "pch.h"
#include "HeightMap.h"

using namespace core;

CHeightMap::CHeightMap(const Eigen::Matrix<float, -1, -1>& vHeightMap)
{
	_ASSERTE(vHeightMap.size() > 0);
	m_Map = vHeightMap;
}

bool CHeightMap::isEmptyValue(int vRow, int vCol)
{
	return __isEmptyValue(vRow, vCol);
}

bool CHeightMap::__isEmptyValue(int vRow, int vCol)
{
	_ASSERTE(vRow >= 0 && vRow < m_Map.rows() && vCol >= 0 && vCol < m_Map.cols());
	return (m_Map(vRow, vCol) == m_Empty);
}

bool CHeightMap::setSize(int vWidth, int vHeight)
{
	_ASSERTE(vWidth > 0 && vHeight > 0);
	m_Map = Eigen::MatrixXf::Constant(vWidth, vHeight, -FLT_MAX);
	return true;
}

bool CHeightMap::setHeightMap(const Eigen::Matrix<float, -1, -1>& vHeightMap)
{
	_ASSERTE(vHeightMap.rows() == m_Map.rows() && vHeightMap.cols() == m_Map.cols());
	m_Map = vHeightMap;
	return true;
}

bool CHeightMap::setValueAt(float vHeight, int vRow, int vCol)
{
	_ASSERTE(vRow >= 0 && vRow < m_Map.rows() && vCol >= 0 && vCol < m_Map.cols());
	m_Map.coeffRef(vRow, vCol) = vHeight;
	return true;
}

bool CHeightMap::setEmptyAt(int vRow, int vCol)
{
	_ASSERTE(vRow >= 0 && vRow < m_Map.rows() && vCol >= 0 && vCol < m_Map.cols());
	m_Map.coeffRef(vRow, vCol) = m_Empty;
	return true;
}

float CHeightMap::getValueAt(int vRow, int vCol) const
{
	_ASSERTE(vRow >= 0 && vRow < m_Map.rows() && vCol >= 0 && vCol < m_Map.cols());
	return m_Map.coeff(vRow, vCol);
}

