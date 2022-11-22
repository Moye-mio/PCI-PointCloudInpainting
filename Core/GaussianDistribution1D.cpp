#include "pch.h"
#include "GaussianDistribution1D.h"

using namespace core;

CGaussianDistribution1D::CGaussianDistribution1D()
	: m_Mean(0.0f)
	, m_Variance(0.0f)
	, m_GaussianCoef(0.0f)
	, m_IsCompute(false)
{}

bool CGaussianDistribution1D::setData(const std::vector<float>& vData)
{
	_ASSERTE(vData.size());

	m_Data = vData;
	return true;
}

bool CGaussianDistribution1D::setChars(float vMean, float vVariance)
{
	_ASSERTE(!std::isnan(vMean));
	_ASSERTE(!std::isnan(vVariance));

	m_IsCompute = true;

	return true;
}

float CGaussianDistribution1D::computeProbability(float vDistribution)
{
	_ASSERTE(m_Data.size());
	_ASSERTE(!std::isnan(vDistribution));

	if (m_IsCompute == false) 
		__computeChars();

	_ASSERTE(m_IsCompute);

	if (m_Variance == 0 && vDistribution == m_Mean)
		return 1.0f;
	else if (m_Variance == 0 && vDistribution != m_Mean)
		return 0.0f;
	else
		return m_GaussianCoef * std::expf(-std::powf(vDistribution - m_Mean, 2) / (2 * m_Variance));
}

void CGaussianDistribution1D::__computeChars()
{
	_ASSERTE(m_Data.size());

	m_Mean = std::accumulate(m_Data.begin(), m_Data.end(), 0.0f) / m_Data.size();
	for (int i = 0; i < m_Data.size(); i++)
		m_Variance += std::powf((m_Data[i] - m_Mean), 2);
	m_Variance /= m_Data.size();

	if (m_Variance != 0)
		m_GaussianCoef = 1 / std::sqrt(2 * std::numbers::pi * m_Variance);

	m_IsCompute = true;
}
