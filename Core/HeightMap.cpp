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
	_ASSERTE(__isIndexValid(vRow, vCol));
	return __isEmptyValue(vRow, vCol);
}

bool CHeightMap::isNeighborhoodEmpty(int vRow, int vCol)
{
	_ASSERTE(__isIndexValid(vRow, vCol));
	if (__isIndexValid(vRow + 1, vCol) && __isEmptyValue(vRow + 1, vCol)) return true;
	if (__isIndexValid(vRow - 1, vCol) && __isEmptyValue(vRow - 1, vCol)) return true;
	if (__isIndexValid(vRow, vCol + 1) && __isEmptyValue(vRow, vCol + 1)) return true;
	if (__isIndexValid(vRow, vCol - 1) && __isEmptyValue(vRow, vCol - 1)) return true;

	return false;
}

bool CHeightMap::__isIndexValid(int vRow, int vCol) const
{
	return (vRow >= 0 && vRow < m_Map.rows() && vCol >= 0 && vCol < m_Map.cols());
}

bool CHeightMap::__isEmptyValue(int vRow, int vCol) const
{
	_ASSERTE(__isIndexValid(vRow, vCol));
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
	_ASSERTE(__isIndexValid(vRow, vCol));
	m_Map.coeffRef(vRow, vCol) = vHeight;
	return true;
}

bool CHeightMap::setEmptyAt(int vRow, int vCol)
{
	_ASSERTE(__isIndexValid(vRow, vCol));
	m_Map.coeffRef(vRow, vCol) = m_Empty;
	return true;
}

float CHeightMap::getValueAt(int vRow, int vCol) const
{
	_ASSERTE(__isIndexValid(vRow, vCol));
	return m_Map.coeff(vRow, vCol);
}

bool CHeightMap::isValid() const
{
	if (m_Map.size()) return true;
	else return false;
}


