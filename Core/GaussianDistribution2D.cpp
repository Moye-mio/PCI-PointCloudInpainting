#include "pch.h"
#include "GaussianDistribution2D.h"

using namespace core;

CGaussianDistribution2D::CGaussianDistribution2D()
	: m_Mean1(0.0f), m_Mean2(0.0f)
	, m_Variance1(0.0f), m_Variance2(0.0f)
	, m_GaussianCoef(0.0f)
	, m_IsCompute(false)
{}

bool CGaussianDistribution2D::setData(const std::vector<float>& vData1, const std::vector<float>& vData2)
{
	_ASSERTE(vData1.size() && vData2.size());

	m_Data1 = vData1;
	m_Data2 = vData2;

	return true;
}

float CGaussianDistribution2D::computeProbability(float vPos1, float vPos2)
{
	_ASSERTE(m_Data1.size() && m_Data2.size());
	_ASSERTE(!std::isnan(vPos1) && !std::isnan(vPos2));

	if (m_IsCompute == false)
		__computeChars();

	_ASSERTE(m_IsCompute);

	if (m_Variance1 == 0 || m_Variance2 == 0)
	{
		if (m_Variance1 == 0 && m_Variance2 == 0 && vPos1 == m_Mean1 && vPos2 == m_Mean2)
			return 1.0f;
		else
			return 0.0f;
	}
	else
		return m_GaussianCoef * std::expf(-0.5 * (std::powf(vPos1 - m_Mean1, 2) / m_Variance1 + std::powf(vPos2 - m_Mean2, 2) / m_Variance2));
	
}

void CGaussianDistribution2D::__computeChars()
{
	_ASSERTE(m_Data1.size() && m_Data2.size());

	m_Mean1 = std::accumulate(m_Data1.begin(), m_Data1.end(), 0.0f) / m_Data1.size();
	m_Mean2 = std::accumulate(m_Data2.begin(), m_Data2.end(), 0.0f) / m_Data2.size();
	for (int i = 0; i < m_Data1.size(); i++)
		m_Variance1 += std::powf((m_Data1[i] - m_Mean1), 2);
	m_Variance1 /= m_Data1.size();
	for (int i = 0; i < m_Data2.size(); i++)
		m_Variance2 += std::powf((m_Data2[i] - m_Mean2), 2);
	m_Variance2 /= m_Data2.size();
	
	if (m_Variance1 != 0 || m_Variance2 == 0)
		m_GaussianCoef = 1 / (2 * std::numbers::pi * std::sqrt(m_Variance1 * m_Variance2));

	m_IsCompute = true;
}
