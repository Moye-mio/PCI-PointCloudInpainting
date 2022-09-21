#include "inpainter2D.h"
#include <opencv2/opencv.hpp>
#include <vector>

Inpainter2D::Inpainter2D(const std::pair<cv::Mat, cv::Mat>& vImages, const cv::Mat& mask, int halfPatchWidth = 4, int mode = 1)
{
	m_X = vImages.first.clone();
	m_Y = vImages.second.clone();
	m_Mask = mask.clone();
	m_WorkX = vImages.first.clone();
	m_WorkY = vImages.second.clone();
	m_Result.create(m_X.size(), m_X.type());
	m_HalfPatchWidth = halfPatchWidth;
}

void PatchMatch(const cv::Mat& SourceImage, const cv::Mat& TargetImage, const cv::Mat& Mask, int nPatchSize, cv::Mat& NearestNeighbor);

cv::Vec3f MeanShift(std::vector<cv::Vec3b> vecVoteColor, std::vector<float> vecVoteWeight, int sigma);

void Inpainter2D::inpaint()
{
	cv::Mat Weights(m_WorkX.size(), CV_32F);
	cv::distanceTransform(m_Mask, Weights, cv::DIST_L2, 3);

	std::srand((int)time(0));

	// 先将mask区域设置为随机，表明该区域无信息
	for (int i = 0; i < m_WorkX.rows; i++)
	{
		for (int j = 0; j < m_WorkX.cols; j++)
		{
			Weights.at<float>(i, j) = (float)pow(1.3, Weights.at<float>(i, j));

			int Random = std::rand() % 255;

			if (m_Mask.at<uchar>(i, j))
			{
				m_X.at<cv::Vec3b>(i, j)[0] = Random;
				m_X.at<cv::Vec3b>(i, j)[1] = Random;
				m_X.at<cv::Vec3b>(i, j)[2] = Random;
			}
		}
	}

	// 多个不同的尺度
	int PyramidNumber = 3;
	cv::Mat CurImage = cv::Mat(), CurMask, CurWeight;
	int PatchSize = 2 * m_HalfPatchWidth + 1;

	float Convergence = 10.0f;

	while (PyramidNumber >= -1)
	{
		float Scale = 1.0f / (1 << PyramidNumber);

		if (PyramidNumber < 0)
		{
			Scale = 1.0f;
			m_HalfPatchWidth /= 2;
			PatchSize = 2 * m_HalfPatchWidth + 1;
		}

		cv::resize(m_X, m_WorkX, cv::Size(m_X.cols * Scale, Scale * m_X.rows));
		cv::resize(m_Y, m_WorkY, cv::Size(m_Y.cols * Scale, Scale * m_Y.rows));
		cv::resize(m_Mask, CurMask, cv::Size(m_Mask.cols * Scale, Scale * m_Mask.rows));
		cv::resize(Weights, CurWeight, cv::Size(Weights.cols * Scale, Scale * Weights.rows));

		if (CurImage.rows * CurImage.cols > 0)
		{
			cv::resize(CurImage, CurImage, cv::Size(m_X.cols * Scale, Scale * m_X.rows));

			// mask 外的区域用上一次的数据填充
			for (int i = 0; i < CurImage.rows; i++)
				for (int j = 0; j < CurImage.cols; j++)
					if (CurMask.at<uchar>(i, j) == 0)
						CurImage.at<Vec3b>(i, j) = m_WorkImage.at<Vec3b>(i, j);
		}
		else
			CurImage = m_WorkImage.clone();

		int IterMaxNumber = 100;
		if (PyramidNumber <= 0)
			IterMaxNumber = 200;

		int MinLength = min(CurImage.rows, CurImage.cols);

		if (MinLength < 2 * PatchSize)
		{
			PyramidNumber--;
			continue;
		}

		// 循环直到本层收敛
		while (true)
		{
			Mat LastImage = CurImage.clone();

			Mat OutPutFrame;
			resize(CurImage, OutPutFrame, Size(m_InputImage.cols, m_InputImage.rows));

			voVideo << OutPutFrame;

			// patchMatch 计算各个patch的最近邻
			Mat NNF;
			PatchMatch(CurImage, CurImage, CurMask, PatchSize, NNF);

			// 遍历图片
			for (int i = 0; i < CurImage.rows; i++)
			{
				for (int k = 0; k < CurImage.cols; k++)
				{
					//需要填充的区域
					if (CurMask.at<uchar>(i, k))
					{
						vector<Vec3b> VoteColor;
						vector<float> VoteWeight;
						vector<float> Dist;

						// 所有经过该点的patch块
						for (int OffsetX = -PatchSize + 1; OffsetX <= 0; OffsetX++)
						{
							for (int OffsetY = -PatchSize + 1; OffsetY <= 0; OffsetY++)
							{
								int StartX = i + OffsetX;
								int StartY = k + OffsetY;

								if (StartX < 0 || StartY < 0 || StartY + PatchSize >= CurImage.cols - 1 || StartX + PatchSize >= CurImage.rows - 1) continue;

								Rect CurPatchRect(StartY, StartX, PatchSize, PatchSize);  // 当前块的
								Mat CurPatch = CurImage(CurPatchRect);

								int NNF_X = NNF.at<Vec3i>(StartX, StartY)[0];
								int NNF_Y = NNF.at<Vec3i>(StartX, StartY)[1];

								Rect NearPatchRect(NNF_X, NNF_Y, PatchSize, PatchSize);  // 当前块的
								Mat NearestPatch = CurImage(NearPatchRect);

								// 两个patch之间的距离
								float Distance = norm(CurPatch, NearestPatch);

								// 给某个颜色投票
								Vec3b Color = NearestPatch.at<Vec3b>(-OffsetX, -OffsetY);

								// 权重
								float Weight = Weights.at<float>(StartX + PatchSize / 2, StartY + PatchSize / 2);

								Dist.push_back(Distance * Distance);
								VoteColor.push_back(Color);
								VoteWeight.push_back(Weight);
							}
						}

						if (VoteWeight.size() < 3)
						{
							continue;
						}

						// 复制一份
						vector<float> DistCopy;
						DistCopy.assign(Dist.begin(), Dist.end());

						// 排序
						sort(DistCopy.begin(), DistCopy.end());

						float Sigma = DistCopy[DistCopy.size() * 3 / 4];

						// 计算权重
						for (int i = 0; i < VoteWeight.size(); i++)
						{
							// 如果sigma不为0
							if (Sigma != 0)
								VoteWeight[i] = VoteWeight[i] * exp(-(Dist[i]) / (2 * Sigma));
						}

						float MaxWeight = 0;
						for (int i = 0; i < VoteWeight.size(); i++)
						{
							if (MaxWeight < VoteWeight[i])
							{
								MaxWeight = VoteWeight[i];
							}
						}

						// 归一化,避免权重过小
						for (int i = 0; i < VoteWeight.size(); i++)
						{
							VoteWeight[i] = VoteWeight[i] / MaxWeight;
						}

						CurImage.at<Vec3b>(i, k) = MeanShift(VoteColor, VoteWeight, 50);
					}
				}
			}
			IterMaxNumber--;

			if (IterMaxNumber <= 0)
			{
				std::cout << "Iter finished..." << std::endl;
				break;
			}

			float Diff = 0;
			int Num = 0;
			for (int i = 0; i < LastImage.rows; i++)
			{
				for (int j = 0; j < LastImage.cols; j++)
				{
					if (CurMask.at<uchar>(i, j))
					{
						Vec3f a1 = LastImage.at<Vec3b>(i, j);
						Vec3f a2 = CurImage.at<Vec3b>(i, j);
						Vec3f a3 = a1 - a2;

						Diff += a3[0] * a3[0] + a3[1] * a3[1] + a3[2] * a3[2];
						Num++;
					}
				}
			}

			Diff = Diff / Num;

			imshow("CurImage", CurImage);
			waitKey(100);

			printf("Pyramid: %d, scale: %f, Diff: %f\n", PyramidNumber, Scale, Diff);

			if (Diff < Convergence)	break;
		}

		// 向下一层，变大一倍
		PyramidNumber--;
		Convergence /= 10.0f;
		if (Convergence < 0.01f)
			Convergence = 0.01f;
	}
	m_Result = CurImage;
}