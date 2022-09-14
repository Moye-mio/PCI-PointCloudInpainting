#include "pch.h"
#include "GradientMapGenerator.h"

using namespace core;

CGradientMapGenerator::CGradientMapGenerator()
	: m_IsConservative(false)
{ }

bool CGradientMapGenerator::generate(const CHeightMap& vHeightMap, bool vIsConservative /*= true*/)
{
	_ASSERTE(vHeightMap.isValid());
	m_HeightMap = vHeightMap;
	m_IsConservative = vIsConservative;

	m_Map.setSize(vHeightMap.getWidth(), vHeightMap.getHeight());
	for (int i = 0; i < m_Map.getWidth(); i++)
		for (int k = 0; k < m_Map.getHeight(); k++)
		{
			if (auto r = __computeGradient(i, k); r.has_value())
				m_Map.setValueAt(r.value(), i, k);
			else
				m_Map.setEmptyAt(i, k);
		}

	return true;
}

std::optional<Eigen::Vector2f> CGradientMapGenerator::__computeGradient(int vRow, int vCol)
{
	_ASSERTE(vRow >= 0 && vRow < m_Map.getWidth() && vCol >= 0 && vCol < m_Map.getHeight());

	if (m_HeightMap.isEmptyValue(vRow, vCol)) return std::nullopt;
	if (m_IsConservative && m_HeightMap.isNeighborhoodEmpty(vRow, vCol)) return std::nullopt;

	// 1.same as self (default); 2. flip; 3. tile
	std::vector<float> Heights;
	if (vRow < m_Map.getWidth() - 1)  Heights.push_back(m_HeightMap.getValueAt(vRow + 1, vCol));
	else							  Heights.push_back(m_HeightMap.getValueAt(vRow, vCol));
	if (vRow > 0)					  Heights.push_back(m_HeightMap.getValueAt(vRow - 1, vCol));
	else							  Heights.push_back(m_HeightMap.getValueAt(vRow, vCol));
	if (vCol < m_Map.getHeight() - 1) Heights.push_back(m_HeightMap.getValueAt(vRow, vCol + 1));
	else							  Heights.push_back(m_HeightMap.getValueAt(vRow, vCol));
	if (vCol > 0)					  Heights.push_back(m_HeightMap.getValueAt(vRow, vCol - 1));
	else							  Heights.push_back(m_HeightMap.getValueAt(vRow, vCol));

	return { { (Heights[0] - Heights[1]) / 2,
			 (Heights[2] - Heights[3]) / 2 } };
}
