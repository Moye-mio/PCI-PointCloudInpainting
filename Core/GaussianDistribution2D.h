#pragma once

namespace core
{
	class CGaussianDistribution2D
	{
	public:
		CGaussianDistribution2D();
		~CGaussianDistribution2D() {}

		[[nodiscard]] bool setData(const std::vector<float>& vData1, const std::vector<float>& vData2);
		[[nodiscard]] bool setChars(float vMean1, float vMean2, float vVariance1, float vVariance2);
		[[nodiscard]] bool isValid() { return m_IsCompute; }
		float computeProbability(float vPos1, float vPos2);

	private:
		void __computeChars(const std::vector<float>& vData1, const std::vector<float>& vData2);

	private:
		float	m_Mean1, m_Mean2;
		float	m_Variance1, m_Variance2;
		float	m_GaussianCoef;
		bool	m_IsCompute;
	};
}