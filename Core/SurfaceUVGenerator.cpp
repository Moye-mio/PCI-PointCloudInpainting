#include "pch.h"
#include "SurfaceUVGenerator.h"

using namespace core;

bool CSurfaceUVGenerator::generateUVSamples(const core::CHeightMap& vMask, int vSPP)
{
	if (__ValidCheck(vMask, vSPP) == false) return false;

	auto SpanPP = Eigen::Vector2f(1.0f / vMask.getWidth(), 1.0f / vMask.getHeight());
	std::vector<Eigen::Vector2f> Samples;
	__generateRandomPoints(vMask.getWidth() * vMask.getHeight() * vSPP, Samples);
	__shiftPoints(Samples, SpanPP, vMask);
	m_Samples = Samples;

	return true;
}

bool CSurfaceUVGenerator::__ValidCheck(const core::CHeightMap& vMask, int vSPP)
{
	_HIVE_EARLY_RETURN(vMask.isValid() == false, "ERROR: Generate new PC by Surface: Raw Image is not Valid", false);
	_HIVE_EARLY_RETURN(vSPP < 1, "ERROR: Generate new PC by Surface: SPP <= 1", false);
	return true;
}

void CSurfaceUVGenerator::__generateRandomPoints(int vTotalSampleNumber, std::vector<Eigen::Vector2f>& voSamples)
{
	std::vector<float> RandomSequence = hiveMath::hiveGenerateRandomRealSet(0.0f, 1.0f, vTotalSampleNumber * 2);
	for (int i = 0; i < RandomSequence.size() / 2; i += 2)
		voSamples.push_back(Eigen::Vector2f(RandomSequence[i], RandomSequence[i + 1]));
}

void CSurfaceUVGenerator::__shiftPoints(std::vector<Eigen::Vector2f>& voSamples, Eigen::Vector2f& vSpanPP, const core::CHeightMap& vMask)
{
	std::vector<Eigen::Vector2f> Copy;

	std::vector<Eigen::Vector2f> Shift = { Eigen::Vector2f(0, 0), Eigen::Vector2f(vSpanPP[0] / 2, 0),  Eigen::Vector2f(0, vSpanPP[1] / 2), Eigen::Vector2f(-vSpanPP[0] / 2, 0),  Eigen::Vector2f(0, -vSpanPP[1] / 2), Eigen::Vector2f(vSpanPP[0] / 2, vSpanPP[1] / 2), Eigen::Vector2f(vSpanPP[0] / 2, -vSpanPP[1] / 2), Eigen::Vector2f(-vSpanPP[0] / 2.0, vSpanPP[1] / 2), Eigen::Vector2f(-vSpanPP[0] / 2, -vSpanPP[1] / 2), };

	for (auto& e : voSamples)
	{
		int Random = hiveMath::hiveGenerateRandomInteger(0, (int)Shift.size() - 1);
		e = Eigen::Vector2f(e[0] + Shift[Random][0], e[1] + Shift[Random][1]);

		if (e[0] < 0 || e[0] > 1 || e[1] < 0 || e[1] > 1) continue;
		Eigen::Vector2i Offset = __calcOffset(e, vSpanPP, Eigen::Vector2i(vMask.getWidth(), vMask.getHeight()));

		if (vMask.getValueAt(Offset[0], Offset[1]) == 1) /* 1 is Empty */
			Copy.emplace_back(e);
	}

	voSamples = Copy;
}

Eigen::Vector2i CSurfaceUVGenerator::__calcOffset(const Eigen::Vector2f& vUV, const Eigen::Vector2f& vSpanPP, const Eigen::Vector2i& vSize)
{
	Eigen::Vector2f Offset(vUV[0] / vSpanPP[0], vUV[1] / vSpanPP[1]);

	if (vSpanPP[0] == 0) Offset[0] = 0;
	if (vSpanPP[1] == 0) Offset[1] = 0;

	if (vSize[0] == Offset[0]) Offset[0] -= 1.0f;
	if (vSize[1] == Offset[1]) Offset[1] -= 1.0f;

	return Offset.cast<int>();
}

