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
		float computeProbability(float vDistribution);

	private:
		void __computeChars();

	private:
		std::vector<float> m_Data;
		float m_Mean;
		float m_Variance;
		float m_GaussianCoef;
		bool m_IsCompute;
	};
}