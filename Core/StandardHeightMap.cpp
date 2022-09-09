#include "pch.h"
#include "StandardHeightMap.h"

using namespace core;

CStandardHeightMap::CStandardHeightMap(float vMaxHeight, float vMinHeight, const Eigen::Matrix<float, -1, -1> vHeightMap)
	: m_MaxHeight(vMaxHeight)
	, m_MinHeight(vMinHeight)
	, m_HeightMap(vHeightMap)
{
	_ASSERTE(vMaxHeight > vMinHeight);
	_ASSERTE(__isHeightMapValid(vHeightMap));
}

void CStandardHeightMap::setMaxHeight(float vHeight)
{
	_ASSERTE(vHeight > m_MinHeight);
	m_MaxHeight = vHeight;
}

void CStandardHeightMap::setMinHeight(float vHeight)
{
	_ASSERTE(vHeight < m_MaxHeight);
	m_MinHeight = vHeight;
}

void CStandardHeightMap::setHeightMap(const Eigen::Matrix<float, -1, -1> vHeightMap)
{
	_ASSERTE(__isHeightMapValid(vHeightMap));

	m_HeightMap = vHeightMap;
}

bool CStandardHeightMap::__isHeightMapValid(const Eigen::Matrix<float, -1, -1> vHeightMap)
{
#ifdef _DEBUG
	_ASSERTE(vHeightMap.size() > 0);
	for (int i = 0; i < vHeightMap.rows(); i++)
		for (int k = 0; k < vHeightMap.cols(); k++)
			_ASSERTE(vHeightMap(i, k) >= 0 && vHeightMap(i, k) <= 1);
#endif // _DEBUG

#ifdef _RELEASE
	if (vHeightMap.size() > 0) return false;
	for (int i = 0; i < vHeightMap.rows(); i++)
		for (int k = 0; k < vHeightMap.cols(); k++)
			if (vHeightMap(i, k) >= 0 && vHeightMap(i, k) <= 1) return false;

#endif // _RELEASE

	return true;
}

void CStandardHeightMap::setHeightAt(float vHeight, int vRow, int vCol)
{
	_ASSERTE(vRow >= 0 && vRow < m_HeightMap.rows() && vCol >= 0 && vCol < m_HeightMap.cols());
	_ASSERTE(vHeight >= 0 && vHeight <= 1);

	m_HeightMap.coeffRef(vRow, vCol) = vHeight;
}


