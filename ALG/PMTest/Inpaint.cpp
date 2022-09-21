#include "pch.h"
#include "Inpaint.h"
#include "PatchMatch.h"
#include "MeanShift.h"

using namespace PM;

CInpainter::CInpainter(const cv::Mat& vRaw, const cv::Mat& vMask, int vPatchSize, int vPyramid, int vMaxIterNumber, float vThreshold)
	: m_Raw(vRaw)
	, m_Mask(vMask)
	, m_PatchSize(vPatchSize)
	, m_Pyramid(vPyramid)
	, m_MaxIterNumber(vMaxIterNumber)
	, m_Threshold(vThreshold)
{}

void CInpainter::inpaint()
{
	cv::VideoWriter VideoWrite;
	__inpaint(false, VideoWrite);
}

void CInpainter::inpaint(cv::VideoWriter& voWritter)
{
	__inpaint(true, voWritter);
}

void CInpainter::__inpaint(bool vIsWriteVideo, cv::VideoWriter& voWritter)
{
	_ASSERTE(m_PatchSize > 0);
	_ASSERTE(m_Raw.data && m_Mask.data);
	_ASSERTE(m_Raw.rows == m_Mask.rows && m_Raw.cols == m_Mask.cols);

	m_Inpainted = m_Raw.clone();
	Eigen::Vector2f ExtremeValue = __computeRange();

	cv::Mat Weights(m_Raw.size(), CV_32F);
	cv::distanceTransform(m_Mask, Weights, cv::DIST_L2, 3);

	for (int i = 0; i < m_Raw.rows; i++)
		for (int k = 0; k < m_Raw.cols; k++)
		{
			Weights.at<float>(i, k) = std::powf(1.3f, -Weights.at<float>(i, k));	/* init weights */

			if (m_Mask.at<unsigned char>(i, k))										/* pixels to be filled */
				m_Inpainted.at<float>(i, k) = hiveMath::hiveGenerateRandomReal(ExtremeValue[1], ExtremeValue[0]);

		}

	cv::Mat Cur = cv::Mat(), Work, CurMask;
	while (m_Pyramid >= 0)															/* different scales */
	{
		float Scale = 1.0f / (1 << m_Pyramid);
		cv::Size Size(m_Raw.cols * Scale, m_Raw.rows * Scale);
		_ASSERTE(Size.width >= 1.0f && Size.height >= 1.0f);

		cv::resize(m_Inpainted, Work, Size);
		cv::resize(m_Mask, CurMask, Size);

		if (Cur.rows * Cur.cols == 0)												/* first loop */
			Cur = Work.clone();
		else
		{
			cv::resize(Cur, Cur, Size);

			for (int i = 0; i < Cur.rows; i++)
				for (int k = 0; k < Cur.cols; k++)
					if (CurMask.at<unsigned char>(i, k) == 0)
						Cur.at<float>(i, k) = Work.at<float>(i, k);
		}

		if (std::min(Cur.rows, Cur.cols) < m_PatchSize * 2)
		{
			m_Pyramid--;
			continue;
		}

		for (int Iter = 0; Iter < m_MaxIterNumber; Iter++)
		{
			cv::Mat Last = Cur.clone();
			cv::Mat NNF = PatchMatch(Cur, Cur, CurMask, m_PatchSize);

			for (int i = 0; i < Cur.rows; i++)										/* traverse image */
				for (int k = 0; k < Cur.cols; k++)
				{
					if (CurMask.at<unsigned char>(i, k) == 0) continue;

					std::vector<float> CandidateColor, CandidateWeight, CandidateDist;
					for (int OffsetX = 1 - m_PatchSize; OffsetX <= 0; OffsetX++)	/* traverse every patch contains this Pixel */
						for (int OffsetY = 1 - m_PatchSize; OffsetY <= 0; OffsetY++)
						{
							int StartX = i + OffsetX;
							int StartY = k + OffsetY;
							if (StartX < 0 || StartY < 0 || StartX + m_PatchSize >= Cur.rows - 1 || StartY + m_PatchSize >= Cur.cols - 1) continue;

							cv::Mat Patch = Cur(cv::Rect(StartY, StartX, m_PatchSize, m_PatchSize));
							cv::Mat NearesetPatch = Cur(cv::Rect(NNF.at<cv::Vec3f>(StartX, StartY)[0], NNF.at<cv::Vec3f>(StartX, StartY)[1], m_PatchSize, m_PatchSize));

							float Dist = std::powf(cv::norm(Patch, NearesetPatch), 2);
							float Color = NearesetPatch.at<float>(-OffsetX, -OffsetY);
							float Weight = Weights.at<float>(StartX + m_PatchSize / 2, StartY + m_PatchSize / 2);

							CandidateDist.push_back(Dist);
							CandidateColor.push_back(Color);
							CandidateWeight.push_back(Weight);
						}

					{																/* MeanShift */
						if (CandidateDist.size() == 0) continue;
						std::vector<float> DistCopy;
						DistCopy.assign(CandidateDist.begin(), CandidateDist.end());
						std::sort(DistCopy.begin(), DistCopy.end());
						float Flag = DistCopy[DistCopy.size() * 3 / 4];
						for (int Index = 0; Index < CandidateWeight.size(); Index++)
							if (Flag != 0)
								CandidateWeight[Index] = CandidateWeight[Index] * std::exp(-CandidateDist[Index] / (2 * Flag));

						float MaxWeight = *std::max_element(CandidateWeight.begin(), CandidateWeight.end());
						_ASSERTE(MaxWeight > 0);
						for (auto& e : CandidateWeight)
							e /= MaxWeight;

						Cur.at<float>(i, k) = MeanShift(CandidateColor, CandidateWeight, 50);
					}
				}

			float Diff = 0.0f;														/* if Diff is too small, it can break early */
			int EmptyNumber = 0;
			for (int i = 0; i < Last.rows; i++)
				for (int k = 0; k < Last.cols; k++)
					if (CurMask.at<unsigned char>(i, k))
					{
						Diff += std::powf(Last.at<float>(i, k) - Cur.at<float>(i, k), 2);
						EmptyNumber++;
					}

			Diff /= EmptyNumber;
			std::cout << "Pyramid: " << m_Pyramid << ", Scale: " << Scale << ", PatchSize: " << m_PatchSize << ", Iter: " << Iter << ", Diff: " << Diff << std::endl;

			if (vIsWriteVideo)
			{
				cv::Mat Show(Cur.size(), CV_8UC1);
				for (int i = 0; i < Cur.rows; i++)
					for (int k = 0; k < Cur.cols; k++)
						Show.at<uchar>(i, k) = Cur.at<float>(i, k);

				cv::Mat OutputFrame;
				cv::resize(Show, OutputFrame, cv::Size(m_Raw.cols, m_Raw.cols));
				voWritter << OutputFrame;
				cv::imshow("Cur", Show);
				cv::waitKey(10);
			}


			if (Diff < m_Threshold) break;
		}

		m_Pyramid--;
		m_Threshold = (m_Threshold > 0.01f) ? (m_Threshold / 10.0f) : 0.01f;
	}

	m_Inpainted = Cur;
}

Eigen::Vector2f CInpainter::__computeRange()
{
	Eigen::Vector2f ExtremeValue(-FLT_MAX, FLT_MAX);	/* Max, Min */
	for (int i = 0; i < m_Raw.rows; i++)
		for (int k = 0; k < m_Raw.cols; k++)
		{
			ExtremeValue[0] = (m_Raw.at<float>(i, k) > ExtremeValue[0]) ? m_Raw.at<float>(i, k) : ExtremeValue[0];
			ExtremeValue[1] = (m_Raw.at<float>(i, k) < ExtremeValue[1]) ? m_Raw.at<float>(i, k) : ExtremeValue[1];
		}

	return ExtremeValue;
}