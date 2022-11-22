#pragma once

namespace core
{
	class CGaussianDistribution2D
	{
	public:
		CGaussianDistribution2D();
		~CGaussianDistribution2D() {}

		[[nodiscard]] bool setData(const std::vector<float>& vData1, const std::vector<float>& vData2);
		float computeProbability(float vPos1, float vPos2);

	private:
		void __computeChars();

	private:
		std::vector<float> m_Data1, m_Data2;
		float m_Mean1, m_Mean2;
		float m_Variance1, m_Variance2;
		float m_GaussianCoef;
		bool m_IsCompute;
	};
}