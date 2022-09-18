#include <opencv2/opencv.hpp>

using namespace cv;

float computePatchDist(const Mat & PatchA, const Mat & PatchB, int PatchSize)
{
	return PatchSize * PatchSize * norm(PatchA, PatchB) ;	// norm(PatchA - PatchB)
}

bool GuessAndImprove(const Mat& vSourceImage, const Mat& vTargetImage, const Mat& vMask, int vCol, int vRow, int vGuessCol, int vGuessRow, int vPatchSize, Mat &vioNearestNeighbor)
{
	// 当前的patch块
	if (vCol == vGuessCol && vRow == vGuessRow)	return false;

	Rect GuessRect(vGuessCol, vGuessRow, vPatchSize, vPatchSize);
	Mat CurMask = vMask(GuessRect);

	int InValidNum = 0;
	for (int Row = 0; Row < CurMask.rows; Row++)
		for (int Col = 0; Col < CurMask.cols; Col++)
			if (CurMask.at<uchar>(Row, Col))
				InValidNum++;

	if (InValidNum * 10 > CurMask.rows * CurMask.cols) return false;

	Rect SourceRect(vCol, vRow, vPatchSize, vPatchSize);

	Mat SourcePatch = vSourceImage(SourceRect);
	Mat TargetPatch = vTargetImage(GuessRect);

	float CurDist = computePatchDist(SourcePatch, TargetPatch, vPatchSize);

	int CurBestDist = vioNearestNeighbor.at<Vec3i>(vRow, vCol)[2];

	if (CurDist < CurBestDist)
	{
		vioNearestNeighbor.at<Vec3i>(vRow, vCol)[0] = vGuessCol;
		vioNearestNeighbor.at<Vec3i>(vRow, vCol)[1] = vGuessRow;
		vioNearestNeighbor.at<Vec3i>(vRow, vCol)[2] = CurDist;
		return true;
	}
	return false;
}

void PatchMatch(const Mat & vSourceImage,const Mat & vTargetImage, const Mat & vMask,  int vPatchSize, Mat & voNearestNeighbor)
{
	// 最近邻数据
	voNearestNeighbor = Mat::zeros(vSourceImage.size(), CV_32SC3);

	int IterNumber = 0;
	int IterMaxNumber = 5;
	int32_t MaxCols = vTargetImage.cols - vPatchSize - 1;
	int32_t MaxRows = vTargetImage.rows - vPatchSize - 1;

	// 先设置随机初值
	for (int Row = 0; Row < vSourceImage.rows - vPatchSize; Row++)
	{
		for (int Col = 0; Col < vSourceImage.cols - vPatchSize; Col++)
		{
			int RandStartCol = rand() % (MaxCols);   // x 坐标
			int RandStartRow = rand() % (MaxRows);   // y 坐标

			voNearestNeighbor.at<Vec3i>(Row, Col)[0] = RandStartCol;
			voNearestNeighbor.at<Vec3i>(Row, Col)[1] = RandStartRow;

			Rect SourceRect(Col, Row, vPatchSize, vPatchSize);
			Mat patchA = vSourceImage(SourceRect);

			Rect TargetRect(RandStartCol, RandStartRow, vPatchSize, vPatchSize);
			Mat patchB = vTargetImage(TargetRect);

			voNearestNeighbor.at<Vec3i>(Row,Col)[2] = computePatchDist(patchA, patchB, vPatchSize);
		}
	}

	while (IterNumber < IterMaxNumber)
	{
		int ColStart = 1;
		int ColEnd = vSourceImage.cols - vPatchSize ;
		int RowStart = 1;
		int RowEnd = vSourceImage.rows - vPatchSize ;
		int Step = 1;

		if (IterNumber % 2)
		{
			ColStart = vSourceImage.cols - vPatchSize - 2;
			ColEnd = -1;
			RowStart = vSourceImage.rows - vPatchSize - 2;
			RowEnd = -1;
			Step = -1;
		}

		int Number = 0;
		for (int Row = RowStart; Row != RowEnd; Row += Step)
		{
			for (int Col = ColStart; Col != ColEnd; Col += Step)
			{
				// 有效范围内
				if (Col - Step < vSourceImage.cols - vPatchSize)
				{
					int GuessCol = voNearestNeighbor.at<Vec3i>(Row, Col - Step)[0] + Step;
					int GuessRow = voNearestNeighbor.at<Vec3i>(Row, Col - Step)[1];

					if (GuessCol < vTargetImage.cols - vPatchSize && GuessCol >= 0)
					{
						// propagation 
						GuessAndImprove(vSourceImage, vTargetImage, vMask, Col, Row, GuessCol, GuessRow, vPatchSize, voNearestNeighbor);
					}

				}
				// 有效范围内
				if (Row - Step < vSourceImage.rows - vPatchSize)
				{
					int nGuessX = voNearestNeighbor.at<Vec3i>(Row - Step, Col)[0];
					int nGuessY = voNearestNeighbor.at<Vec3i>(Row - Step, Col)[1] + Step;

					if (nGuessY < vTargetImage.rows - vPatchSize && nGuessY >= 0)
					{
						// propagation 
						GuessAndImprove(vSourceImage, vTargetImage, vMask, Col, Row, nGuessX, nGuessY, vPatchSize, voNearestNeighbor);
					}
				}

				// random guess
				int rs_start = max(vTargetImage.rows,  vTargetImage.cols);
				int nBestX = voNearestNeighbor.at<Vec3i>(Row, Col)[0];
				int nBestY = voNearestNeighbor.at<Vec3i>(Row, Col)[1];

				for (int mag = rs_start; mag >= 1; mag /= 2) 
				{
					/* Sampling window */
					int xmin = max(nBestX - mag, 0), xmax = min(nBestX + mag + 1, vTargetImage.cols - vPatchSize - 1);
					int ymin = max(nBestY - mag, 0), ymax = min(nBestY + mag + 1, vTargetImage.rows - vPatchSize - 1);

					int xp = xmin + rand() % (xmax - xmin);
					int yp = ymin + rand() % (ymax - ymin);

					Number += GuessAndImprove(vSourceImage, vTargetImage, vMask, Col, Row, xp, yp,  vPatchSize, voNearestNeighbor);
				}
			}
		}
		IterNumber++;
	}
}