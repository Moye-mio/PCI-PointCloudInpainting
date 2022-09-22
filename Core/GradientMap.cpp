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

bool CGradientMap::setValueAt(float vGradient, int vRow, int vCol, int vAxis)
{
	_ASSERTE(vRow >= 0 && vRow < m_Map.rows() && vCol >= 0 && vCol < m_Map.cols());
	_ASSERTE(vAxis == 0 || vAxis == 1);

	m_Map.coeffRef(vRow, vCol)[vAxis] = vGradient;
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

bool CGradientMap::isValid() const
{
	if (!m_Map.size()) return false;
	for (int i = 0; i < m_Map.rows(); i++)
		for (int k = 0; k < m_Map.cols(); k++)
		{
			if (m_Map(i, k)[0] == m_Empty[0] && m_Map(i, k)[1] != m_Empty[1])	return false;
			if (m_Map(i, k)[1] == m_Empty[1] && m_Map(i, k)[0] != m_Empty[0])	return false;
		}
	return true;
}

void CGradientMap::generateMask(CHeightMap& voMap)
{
	_ASSERTE(this->isValid());

	voMap.setSize(m_Map.rows(), m_Map.rows());
	for (int i = 0; i < m_Map.rows(); i++)
		for (int k = 0; k < m_Map.rows(); k++)
		{
			if (this->__isEmptyValue(i, k))		voMap.setValueAt(1, i, k);
			else								voMap.setValueAt(0, i, k);
		}
}

Eigen::MatrixXf CGradientMap::getDataInX() const
{
	Eigen::MatrixXf Data(m_Map.rows(), m_Map.cols());
	for (int i = 0; i < m_Map.rows(); i++)
		for (int k = 0; k < m_Map.cols(); k++)
			Data.coeffRef(i, k) = m_Map(i, k)[0];
	return Data;
}

Eigen::MatrixXf CGradientMap::getDataInY() const
{
	Eigen::MatrixXf Data(m_Map.rows(), m_Map.cols());
	for (int i = 0; i < m_Map.rows(); i++)
		for (int k = 0; k < m_Map.cols(); k++)
			Data.coeffRef(i, k) = m_Map(i, k)[1];
	return Data;
}
