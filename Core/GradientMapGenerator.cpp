#include "pch.h"
#include "GradientMapGenerator.h"

using namespace core;

CGradientMapGenerator::CGradientMapGenerator()
	: m_IsConservative(false)
{ }

bool CGradientMapGenerator::generate(const CHeightMap& vHeightMap, bool vIsConservative /*= true*/)
{
	_ASSERTE(vHeightMap.isValid());
	m_Map.setSize(vHeightMap.getWidth(), vHeightMap.getHeight());
	m_IsConservative = vIsConservative;

	__generate(vHeightMap, 0);
	__generate(vHeightMap, 1);

	return true;
}

bool CGradientMapGenerator::generate(const CGradientMap& vGradientMap, bool vIsConservative /*= true*/)
{
	_ASSERTE(vGradientMap.isValid());
	m_Map.setSize(vGradientMap.getWidth(), vGradientMap.getHeight());
	m_IsConservative = vIsConservative;

	core::CHeightMap HeightMapX;
	HeightMapX.setHeightMap(vGradientMap.getDataInX());
	_ASSERTE(HeightMapX.isValid());

	core::CHeightMap HeightMapY;
	HeightMapY.setHeightMap(vGradientMap.getDataInY());
	_ASSERTE(HeightMapY.isValid());

	__generate(HeightMapX, 0);
	__generate(HeightMapY, 1);

	return true;
}

bool CGradientMapGenerator::__generate(const CHeightMap& vHeightMap, int vAxis)
{
	for (int i = 0; i < vHeightMap.getWidth(); i++)
		for (int k = 0; k < vHeightMap.getHeight(); k++)
		{
			if (auto r = __computeGradient(vHeightMap, i, k, vAxis); r.has_value())
				m_Map.setValueAt(r.value(), i, k, vAxis);
			else
				m_Map.setEmptyAt(i, k);
		}
	return true;
}

std::optional<float> CGradientMapGenerator::__computeGradient(const CHeightMap& vHeightMap, int vRow, int vCol, int vMode)
{
	_ASSERTE(vRow >= 0 && vRow < vHeightMap.getWidth() && vCol >= 0 && vCol < vHeightMap.getHeight());

	if (vHeightMap.isEmptyValue(vRow, vCol)) return std::nullopt;
	if (m_IsConservative && vHeightMap.isNeighborhoodEmpty(vRow, vCol)) return std::nullopt;

	// 1.same as self (default); 2. flip; 3. tile
	std::vector<float> Heights;

	if (vMode == 0)											/* x axis / row */
	{
		if (vRow < vHeightMap.getWidth() - 1)				Heights.push_back(vHeightMap.getValueAt(vRow + 1, vCol));
		else if (vRow >= 1)									Heights.push_back(2 * vHeightMap.getValueAt(vRow, vCol) - vHeightMap.getValueAt(vRow - 1, vCol));
		if (vRow > 0)										Heights.push_back(vHeightMap.getValueAt(vRow - 1, vCol));
		else if (vRow <= vHeightMap.getWidth() - 2)			Heights.push_back(2 * vHeightMap.getValueAt(vRow, vCol) - vHeightMap.getValueAt(vRow + 1, vCol));
	}
	else if (vMode == 1)									/* y axis / row */
	{
		if (vCol < vHeightMap.getHeight() - 1)				Heights.push_back(vHeightMap.getValueAt(vRow, vCol + 1));
		else if (vCol >= 1)									Heights.push_back(2 * vHeightMap.getValueAt(vRow, vCol) - vHeightMap.getValueAt(vRow, vCol - 1));
		if (vCol > 0)										Heights.push_back(vHeightMap.getValueAt(vRow, vCol - 1));
		else if (vCol <= vHeightMap.getHeight() - 2)		Heights.push_back(2 * vHeightMap.getValueAt(vRow, vCol) - vHeightMap.getValueAt(vRow, vCol + 1));
	}

	if (Heights.size() < 2) return std::nullopt;

	return { (Heights[0] - Heights[1]) / 2 };
}


