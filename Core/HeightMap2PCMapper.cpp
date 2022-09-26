#include "pch.h"
#include "HeightMap2PCMapper.h"

#include "Interpolation.h"

using namespace core;

void CHeightMap2PCMapper::map2PC(PC_t::Ptr& voCloud, const std::pair<CHeightMap, CHeightMap>& vMaps, const SAABB& vBox, int vPointNumberPerPixel)
{
	_ASSERTE(vMaps.first.isValid() && vMaps.second.isValid());
	_ASSERTE(vMaps.first.getWidth() == vMaps.second.getWidth() && vMaps.first.getHeight() == vMaps.second.getHeight());
	_ASSERTE(vBox.isValid());
	_ASSERTE(vPointNumberPerPixel > 0);

	m_RawMap = vMaps.first;
	m_FilledMap = vMaps.second;
	m_Box = vBox;
	m_SpanPerPixel = std::make_pair<float, float>((m_Box._Max[0] - m_Box._Min[0]) / m_RawMap.getWidth(), (m_Box._Max[1] - m_Box._Min[1]) / m_RawMap.getHeight());

	std::vector<std::pair<float, float>> RandomPoints;
	__generateRandomPoints(m_RawMap.getWidth() * m_RawMap.getHeight() * vPointNumberPerPixel, RandomPoints);
	__map2NewPoints(voCloud, RandomPoints);
}

void CHeightMap2PCMapper::__generateRandomPoints(int vNumber, std::vector<std::pair<float, float>>& voRandomPoints)
{
	std::pair<float, float> Span = std::make_pair<float, float>(m_Box._Max[0] - m_Box._Min[0], m_Box._Max[1] - m_Box._Min[1]);
	/*const std::vector<unsigned int> BaseSet = { 2, 7 };
	std::vector<float> HaltonSequence = hiveMath::hiveGenerateHaltonSequence(BaseSet, vNumber);

	_ASSERTE(HaltonSequence.size() == vNumber * BaseSet[0]);

	for (int i = 0; i < HaltonSequence.size() / 2; i += 2)
	{
		voRandomPoints.push_back(std::make_pair(HaltonSequence[i] * Span.first + m_Box._Min[0], HaltonSequence[i + 1] * Span.second + m_Box._Min[1]));
	}*/

	std::vector<float> RandomSequence = hiveMath::hiveGenerateRandomRealSet(0.0f, 1.0f, vNumber * 2);
	for (int i = 0; i < RandomSequence.size() / 2; i += 2)
	{
		voRandomPoints.push_back(std::make_pair(RandomSequence[i] * Span.first + m_Box._Min[0], RandomSequence[i + 1] * Span.second + m_Box._Min[1]));
	}
}

void CHeightMap2PCMapper::__map2NewPoints(PC_t::Ptr& voCloud, const std::vector<std::pair<float, float>>& vPoints)
{
	if (voCloud == nullptr) voCloud = std::make_shared<PC_t>();
	else voCloud->clear();

	std::vector<std::pair<float, float>> Shift = { std::make_pair(0, 0), std::make_pair(m_SpanPerPixel.first / 2, 0),  std::make_pair(0, m_SpanPerPixel.second / 2),
	std::make_pair(-m_SpanPerPixel.first / 2, 0),  std::make_pair(0, -m_SpanPerPixel.second / 2), std::make_pair(m_SpanPerPixel.first / 2, m_SpanPerPixel.second / 2),
	std::make_pair(m_SpanPerPixel.first / 2, -m_SpanPerPixel.second / 2), std::make_pair(-m_SpanPerPixel.first / 2.0, m_SpanPerPixel.second / 2),
	std::make_pair(-m_SpanPerPixel.first / 2, -m_SpanPerPixel.second / 2), };

	for (auto& e : vPoints)
	{
		int Random = hiveMath::hiveGenerateRandomInteger(0, (int)Shift.size() - 1);
		auto OffsetCoor = __computeCoor(std::make_pair(e.first + Shift[Random].first, e.second + Shift[Random].second), true);

		if (OffsetCoor.first < 0 || OffsetCoor.first >= m_RawMap.getWidth() || OffsetCoor.second < 0 || OffsetCoor.second >= m_RawMap.getHeight())	continue;

		if (m_RawMap.isEmptyValue((int)OffsetCoor.first, (int)OffsetCoor.second))
		{
			auto Coor = __computeCoor(e, false);
			voCloud->emplace_back(Point_t(e.first, e.second, m_FilledMap.Sample(Coor)));
		}
	}
}

std::pair<float, float> CHeightMap2PCMapper::__computeCoor(const std::pair<float, float>& vPoint, bool vTune)
{
	float OffsetX = (vPoint.first - m_Box._Min[0]) / (m_Box._Max[0] - m_Box._Min[0]) * m_RawMap.getWidth();
	float OffsetY = (vPoint.second - m_Box._Min[1]) / (m_Box._Max[1] - m_Box._Min[1]) * m_RawMap.getHeight();

	if (m_Box._Max[0] == m_Box._Min[0]) OffsetX = 0;
	if (m_Box._Max[1] == m_Box._Min[1]) OffsetY = 0;

	if (vTune)
	{
		if (OffsetX == m_RawMap.getWidth()) OffsetX--;
		if (OffsetY == m_RawMap.getHeight()) OffsetY--;
	}

	return std::make_pair(OffsetX, OffsetY);
}
