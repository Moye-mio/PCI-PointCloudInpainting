#include "pch.h"
#include "SolverBuilder.h"

using namespace dataManagement;

bool CSolverBuilder::setUp(const core::CHeightMap& vRaw, const core::CGradientMap& vGoG, const core::CGradientMap& vGradient)
{
	_ASSERTE(vRaw.isValid() && vGoG.isValid() && vGradient.isValid());
	_ASSERTE(vGoG.isNoEmptyValue() && vGradient.isNoEmptyValue());
	m_Raw = vRaw;
	m_GoG = vGoG;
	m_Gradient = vGradient;
	__init();

	for (int i = 0; i < m_Unknowns.size(); i++)
		__setUpEachEquation(i);
	return true;
}

void CSolverBuilder::__init()
{
	__buildMask();
	__countUnknowns();
	__initMatrix();
}

void CSolverBuilder::__buildMask()
{
	m_Raw.generateMask(m_Mask);

	_ASSERTE(m_Mask.isValid());
}

void CSolverBuilder::__countUnknowns()
{
	for (int i = 0; i < m_Mask.getWidth(); i++)
		for (int k = 0; k < m_Mask.getHeight(); k++)
			if (m_Mask.getValueAt(i, k) == 1)
				m_Unknowns.push_back({ i, k });
	_ASSERTE(m_Unknowns.size());
}

void CSolverBuilder::__initMatrix()
{
	m_Coeff = Eigen::MatrixXf(m_Unknowns.size(), m_Unknowns.size());
	m_Const = Eigen::MatrixXf(m_Unknowns.size(), 1);
	m_Coeff.setZero();
	m_Const.setZero();
}

void CSolverBuilder::__setUpEachEquation(int vIndex)
{
	_ASSERTE(vIndex >= 0 && vIndex < m_Unknowns.size());
	int X = m_Unknowns[vIndex][0];
	int Y = m_Unknowns[vIndex][1];
	_ASSERTE(m_Mask.getValueAt(X, Y));
	
	if ((X == 0 || X == m_Mask.getWidth() - 1) && (Y == 0 || Y == m_Mask.getHeight() - 1))			/* Corner */
	{
		float ConstNumber = m_Gradient.getValueAt(X, Y)[0] + m_Gradient.getValueAt(X, Y)[1];
		ConstNumber = __handleNeighbor({ X, Y - 1 }, ConstNumber, vIndex, 1);
		ConstNumber = __handleNeighbor({ X - 1, Y }, ConstNumber, vIndex, 1);
		ConstNumber = __handleNeighbor({ X + 1, Y }, ConstNumber, vIndex, 1);
		ConstNumber = __handleNeighbor({ X, Y + 1 }, ConstNumber, vIndex, 1);
		m_Coeff.coeffRef(vIndex, vIndex) = -2;
		m_Const.coeffRef(vIndex, 0) = ConstNumber;

	}
	else if (X == 0 || X == m_Mask.getWidth() - 1 || Y == 0 || Y == m_Mask.getHeight() - 1)			/* Border */
	{
		float ConstNumber = m_GoG.getValueAt(X, Y)[0] + m_GoG.getValueAt(X, Y)[1];
		if (X == 0 || X == m_Mask.getWidth() - 1)
		{
			ConstNumber = __handleNeighbor({ X, Y - 1 }, ConstNumber, vIndex, 1);
			ConstNumber = __handleNeighbor({ X, Y + 1 }, ConstNumber, vIndex, 1);

		}
		else
		{
			ConstNumber = __handleNeighbor({ X - 1, Y }, ConstNumber, vIndex, 1);
			ConstNumber = __handleNeighbor({ X + 1, Y }, ConstNumber, vIndex, 1);
		}

		m_Coeff.coeffRef(vIndex, vIndex) = -2;
		m_Const.coeffRef(vIndex, 0) = ConstNumber;
	}
	else
	{
		// D(x,y-1)+D(x-1,y)+D(x+1,y)+D(x,y+1)-4D(x,y)=GoG(x,y)[0]+GoG(x,y)[1]
		float ConstNumber = m_GoG.getValueAt(X, Y)[0] + m_GoG.getValueAt(X, Y)[1];
		ConstNumber = __handleNeighbor({ X, Y - 1 }, ConstNumber, vIndex, 1);
		ConstNumber = __handleNeighbor({ X - 1, Y }, ConstNumber, vIndex, 1);
		ConstNumber = __handleNeighbor({ X + 1, Y }, ConstNumber, vIndex, 1);
		ConstNumber = __handleNeighbor({ X, Y + 1 }, ConstNumber, vIndex, 1);
		m_Coeff.coeffRef(vIndex, vIndex) = -4;
		m_Const.coeffRef(vIndex, 0) = ConstNumber;
	}

}

std::optional<int> CSolverBuilder::__findUnknownIndex(const Eigen::Vector2f& vPos)
{
	_ASSERTE(m_Unknowns.size());

	auto Iter = std::find(m_Unknowns.begin(), m_Unknowns.end(), vPos);
	if (Iter == m_Unknowns.end()) return std::nullopt;
	else return std::distance(m_Unknowns.begin(), Iter);
}

float CSolverBuilder::__handleNeighbor(const Eigen::Vector2f& vPos, float vConstNumber, int vIndex, float vCoeff)
{
	if (vPos[0] < 0 || vPos[0] > m_Mask.getWidth() - 1 || vPos[1] < 0 || vPos[1] > m_Mask.getHeight() - 1)
		return vConstNumber;

	if (m_Mask.getValueAt(vPos[0], vPos[1]) == 0)
		return vConstNumber - m_Raw.getValueAt(vPos[0], vPos[1]);
	else
	{
		auto r = __findUnknownIndex(Eigen::Vector2f(vPos[0], vPos[1]));
		_ASSERTE(r.has_value());
		m_Coeff.coeffRef(vIndex, r.value()) = vCoeff;
	}
	return vConstNumber;
}

