#include "pch.h"
#include "GradientMap.h"

using namespace core;

CGradientMap::CGradientMap(const Eigen::Matrix<Eigen::Vector2f, -1, -1>& vGradientMap)
{
	_ASSERTE(vGradientMap.size() > 0);
	m_Map = vGradientMap;
}

bool CGradientMap::isEmptyValue(int vRow, int vCol)
{
	return __isEmptyValue(vRow, vCol);
}

bool CGradientMap::__isEmptyValue(int vRow, int vCol)
{
	_ASSERTE(vRow >= 0 && vRow < m_Map.rows() && vCol >= 0 && vCol < m_Map.cols());
	return (m_Map(vRow, vCol) == m_Empty);
}

bool CGradientMap::setSize(int vWidth, int vHeight)
{
	_ASSERTE(vWidth > 0 && vHeight > 0);
	m_Map.setConstant(vWidth, vHeight, m_Empty);
	return true;
}

bool CGradientMap::setGradientMap(const Eigen::Matrix<Eigen::Vector2f, -1, -1>& vGradientMap)
{
	_ASSERTE(vGradientMap.rows() == m_Map.rows() && vGradientMap.cols() == m_Map.cols());
	m_Map = vGradientMap;
	return true;
}

bool CGradientMap::setValueAt(Eigen::Vector2f vGradient, int vRow, int vCol)
{
	_ASSERTE(vRow >= 0 && vRow < m_Map.rows() && vCol >= 0 && vCol < m_Map.cols());
	m_Map.coeffRef(vRow, vCol) = vGradient;
	return true;
}

bool CGradientMap::setEmptyAt(int vRow, int vCol)
{
	_ASSERTE(vRow >= 0 && vRow < m_Map.rows() && vCol >= 0 && vCol < m_Map.cols());
	m_Map.coeffRef(vRow, vCol) = m_Empty;
	return true;
}

Eigen::Vector2f CGradientMap::getValueAt(int vRow, int vCol) const
{
	_ASSERTE(vRow >= 0 && vRow < m_Map.rows() && vCol >= 0 && vCol < m_Map.cols());
	return m_Map.coeff(vRow, vCol);
}

