#include "pch.h"
#include "MeanShift.h"

using namespace PM;

float PM::MeanShift(const std::vector<float>& vColors, const std::vector<float>& vWeights, int vSigma)
{
	_ASSERTE(vColors.size() * vWeights.size() > 0);
	float Mean = 0.0f;
	float TotalWeight = 0.0f;
	for (int i = 0; i < vColors.size(); i++)
	{
		_ASSERTE(vWeights[i] >= 0);
		Mean += vColors[i] * vWeights[i];
		TotalWeight += vWeights[i];
	}
	_ASSERTE(TotalWeight > 0);
	Mean /= TotalWeight;

	for (int i = 0; i < 5; i++)
	{
		float Scale = 3 / (1 << i);		/* from 3 to 0.2 */
		float Thresh = std::powf(Scale * vSigma, 2);
		for (int Iter = 0; Iter < 12; Iter++)
		{
			float CurMean = 0.0f;
			int GroupNumber = 0;
			TotalWeight = 0.0f;

			for (int k = 0; k < vColors.size(); k++)
				if (std::powf(vColors[k] - Mean, 2) < Thresh / 3)
				{
					CurMean += vColors[k] * vWeights[k];
					TotalWeight += vWeights[k];
					GroupNumber++;
				}

			if (GroupNumber == 0) break;

			_ASSERTE(TotalWeight > 0);
			CurMean /= TotalWeight;
			if (std::powf(CurMean - Mean, 2) * 3 < 10) break;

			Mean = CurMean;
		}
	}

	return Mean;
}

cv::Vec2f PM::MeanShift(const std::vector<cv::Vec2f>& vColors, const std::vector<float>& vWeights, int vSigma)
{
	_ASSERTE(vColors.size() * vWeights.size() > 0);
	cv::Vec2f Mean = { 0.0f, 0.0f };
	float TotalWeight = 0.0f;
	for (int i = 0; i < vColors.size(); i++)
	{
		_ASSERTE(vWeights[i] >= 0);
		Mean += vColors[i] * vWeights[i];
		TotalWeight += vWeights[i];
	}
	_ASSERTE(TotalWeight > 0);
	Mean /= TotalWeight;

	for (int i = 0; i < 5; i++)
	{
		float Scale = 3 / (1 << i);		/* from 3 to 0.2 */
		float Thresh = std::powf(Scale * vSigma, 2);
		for (int Iter = 0; Iter < 12; Iter++)
		{
			cv::Vec2f CurMean = { 0.0f, 0.0f };
			int GroupNumber = 0;
			TotalWeight = 0.0f;

			for (int k = 0; k < vColors.size(); k++)
				if (std::powf(vColors[k][0] - Mean[0], 2) + std::powf(vColors[k][1] - Mean[1], 2) < Thresh / 3 * 2)
				{
					CurMean += vColors[k] * vWeights[k];
					TotalWeight += vWeights[k];
					GroupNumber++;
				}

			if (GroupNumber == 0) break;

			_ASSERTE(TotalWeight > 0);
			CurMean /= TotalWeight;
			if (std::powf(CurMean[0] - Mean[0], 2) + std::powf(CurMean[1] - Mean[1], 2) < 20 / 3) break;

			Mean = CurMean;
		}
	}

	return Mean;
}