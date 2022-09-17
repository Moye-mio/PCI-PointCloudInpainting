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
	_ASSERTE(vHeightMap.size() > 0);
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

float CHeightMap::Sample(const std::pair<float, float>& vCoor)
{
	_ASSERTE(__isIndexValid(vCoor.first, vCoor.second));

	std::pair<float, float> NewCoor;
	NewCoor.first = std::clamp(vCoor.first - 0.5f, 0.0f, (float)m_Map.rows() - 1.0f);
	NewCoor.second = std::clamp(vCoor.second - 0.5f, 0.0f, (float)m_Map.cols() - 1.0f);

	std::pair<int, int> StartIndex = NewCoor;
	if (NewCoor.first == m_Map.rows() - 1)	StartIndex.first = NewCoor.first - 1;
	if (NewCoor.second == m_Map.cols() - 1)	StartIndex.second = NewCoor.second - 1;

	std::vector<float> Values;
	Values.push_back(m_Map.coeff(StartIndex.first, StartIndex.second));
	Values.push_back(m_Map.coeff(StartIndex.first, StartIndex.second + 1));
	Values.push_back(m_Map.coeff(StartIndex.first + 1, StartIndex.second));
	Values.push_back(m_Map.coeff(StartIndex.first + 1, StartIndex.second + 1));

	float Height = common::bilinearInterpolate(std::make_pair(NewCoor.first - (float)StartIndex.first, NewCoor.second - (float)StartIndex.second), Values);
	return Height;
}

