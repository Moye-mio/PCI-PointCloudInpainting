#pragma once
#include "HeightMap.h"

namespace core
{
	class CSurfaceUVGenerator
	{
	public:
		CSurfaceUVGenerator() = default;
		~CSurfaceUVGenerator() = default;

		bool generateUVSamples(const core::CHeightMap& vMask, int vSPP);
		void dumpSamples(std::vector<Eigen::Vector2f>& voSamples) { voSamples = m_Samples; }

	private:
		bool __ValidCheck(const core::CHeightMap& vMask, int vSPP);
		void __generateRandomPoints(int vTotalSampleNumber, std::vector<Eigen::Vector2f>& voSamples);
		void __shiftPoints(std::vector<Eigen::Vector2f>& voSamples, Eigen::Vector2f& vSpanPP, const core::CHeightMap& vMask);
		Eigen::Vector2i __calcOffset(const Eigen::Vector2f& vUV, const Eigen::Vector2f& vSpanPP, const Eigen::Vector2i& vSize);

	private:
		std::vector<Eigen::Vector2f> m_Samples;
	};
}