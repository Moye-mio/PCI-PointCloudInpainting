#pragma once

namespace core
{
	class CGaussianDistribution1D
	{
	public:
		CGaussianDistribution1D();
		~CGaussianDistribution1D() {}

		[[nodiscard]] bool setData(const std::vector<float>& vData);
		[[nodiscard]] bool setChars(float vMean, float vVariance);
		[[nodiscard]] bool isValid() { return m_IsCompute; }
		float computeProbability(float vDistribution);

	private:
		void __computeChars(const std::vector<float>& vData);

	private:
		float	m_Mean;
		float	m_Variance;
		float	m_GaussianCoef;
		bool	m_IsCompute;
	};
}