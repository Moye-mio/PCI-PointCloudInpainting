#include "pch.h"
#include "PatchMatch.h"

using namespace PM;

/*
  opencv coordinates:
  ------------------->
  |					 x(col)
  |
  |
  |
  ¡ý y(row)
*/

float computePatchDist(const cv::Mat& vLhs, const cv::Mat& vRhs, int vPatchSize)
{
	return cv::norm(vLhs, vRhs) * vPatchSize * vPatchSize;
}

void propagate(const cv::Mat& vSource, const cv::Mat& vTarget, const cv::Mat& vMask, const std::pair<int, int>& vPos /* row, col */, const std::pair<int, int>& vPosGuess /* row, col */, int vPatchSize, cv::Mat vioNN)
{
	if (vPos.first == vPosGuess.first && vPos.second == vPosGuess.second) return;

	cv::Rect TargetRect(vPosGuess.second, vPosGuess.first, vPatchSize, vPatchSize);
	cv::Mat MaskPatch = vMask(TargetRect);

	int InvalidNumber = 0;
	for (int i = 0; i < MaskPatch.rows; i++)
		for (int k = 0; k < MaskPatch.cols; k++)
			if (MaskPatch.at<unsigned char>(i, k))
				InvalidNumber++;
	if (InvalidNumber * 10 > MaskPatch.rows * MaskPatch.cols) return;

	cv::Mat SourcePatch = vSource(cv::Rect(vPos.second, vPos.first, vPatchSize, vPatchSize));
	cv::Mat TargetPatch = vTarget(TargetRect);
	float Dist = computePatchDist(SourcePatch, TargetPatch, vPatchSize);
	if (Dist < vioNN.at<cv::Vec3f>(vPos.first, vPos.second)[2])
	{
		vioNN.at<cv::Vec3f>(vPos.first, vPos.second)[0] = vPosGuess.second;
		vioNN.at<cv::Vec3f>(vPos.first, vPos.second)[1] = vPosGuess.first;
		vioNN.at<cv::Vec3f>(vPos.first, vPos.second)[2] = Dist;
	}
}


cv::Mat PM::PatchMatch(const cv::Mat& vSource, const cv::Mat& vTarget, const cv::Mat& vMask, int vPatchSize)
{
	cv::Mat NN = cv::Mat::zeros(vSource.size(), CV_32FC3);		/* Col, Row, Dist */

	for (int i = 0; i < vSource.rows - vPatchSize; i++)
		for (int k = 0; k < vSource.cols - vPatchSize; k++)
		{
			NN.at<cv::Vec3f>(i, k)[0] = hiveMath::hiveGenerateRandomInteger(0, vTarget.cols - vPatchSize - 1);
			NN.at<cv::Vec3f>(i, k)[1] = hiveMath::hiveGenerateRandomInteger(0, vTarget.rows - vPatchSize - 1);

			cv::Mat SourcePatch = vSource(cv::Rect(k, i, vPatchSize, vPatchSize));
			cv::Mat TargetPatch = vTarget(cv::Rect(NN.at<cv::Vec3f>(i, k)[0], NN.at<cv::Vec3f>(i, k)[1], vPatchSize, vPatchSize));

			NN.at<cv::Vec3f>(i, k)[2] = computePatchDist(SourcePatch, TargetPatch, vPatchSize);
		}

	for (int Iter = 0; Iter < 5; Iter++)
	{
		int RowStart, RowEnd, ColStart, ColEnd, Step;
		if (Iter % 2)
		{
			RowStart = vSource.rows - vPatchSize - 2;
			RowEnd = -1;
			ColStart = vSource.cols - vPatchSize - 2;
			ColEnd = -1;
			Step = -1;
		}
		else
		{
			RowStart = 1;
			RowEnd = vSource.rows - vPatchSize;
			ColStart = 1;
			ColEnd = vSource.cols - vPatchSize;
			Step = 1;
		}

		for (int i = RowStart; i != RowEnd; i += Step)
			for (int k = ColStart; k != ColEnd; k += Step)
			{
				if (k < vSource.cols - vPatchSize + Step)
				{
					std::pair<int, int> PosGuess(NN.at<cv::Vec3f>(i, k - Step)[1], NN.at<cv::Vec3f>(i, k - Step)[0] + Step);	 /* row, col */

					if (PosGuess.second < vTarget.cols - vPatchSize && PosGuess.second >= 0)
						propagate(vSource, vTarget, vMask, std::make_pair(i, k), PosGuess, vPatchSize, NN);
				}

				if (i < vSource.rows - vPatchSize + Step)
				{
					std::pair<int, int> PosGuess(NN.at<cv::Vec3f>(i - Step, k)[1] + Step, NN.at<cv::Vec3f>(i - Step, k)[0]);	 /* row, col */

					if (PosGuess.first < vTarget.rows - vPatchSize && PosGuess.first >= 0)
						propagate(vSource, vTarget, vMask, std::make_pair(i, k), PosGuess, vPatchSize, NN);
				}

				int CurX = NN.at<cv::Vec3f>(i, k)[0];
				int CurY = NN.at<cv::Vec3f>(i, k)[1];

				for (int Range = std::max(vTarget.rows, vTarget.cols); Range >= 1; Range /= 2)
				{
					int RandomRow = hiveMath::hiveGenerateRandomInteger(std::max(CurY - Range, 0), std::min(CurY + Range + 1, vTarget.rows - vPatchSize - 1));
					int RandomCol = hiveMath::hiveGenerateRandomInteger(std::max(CurX - Range, 0), std::min(CurX + Range + 1, vTarget.cols - vPatchSize - 1));

					propagate(vSource, vTarget, vMask, std::make_pair(i, k), std::make_pair(RandomRow, RandomCol), vPatchSize, NN);
				}
			}
	}

	return NN;
}
