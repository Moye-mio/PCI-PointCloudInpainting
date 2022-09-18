#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

Vec3f MeanShift(const vector<Vec3b> vColors, const vector<float> vWeights, int sigma)
{
	Vec3f Mean3f = Vec3f{0,0,0};

	float TotalWeight = 0.0f;
	for (int i = 0; i < vColors.size(); i++)
	{
		Mean3f += (Vec3f)(vColors[i] * vWeights[i]);
		TotalWeight += vWeights[i];
	}

	Mean3f = Mean3f / TotalWeight;

	for (int t = 0; t < 5; t++)
	{
		float scale = 3 / (1 << t);// ·ù¶ÈÓÉ3sigmaµ½0.2sigma

		float Thresh = (scale * sigma) * (scale * sigma);

		int nIterNum = 0;
		while (1)
		{
			Vec3f CurMean3f = Vec3f{ 0,0,0 };
			int GroupNum = 0;

			TotalWeight = 0.0f;
			for (int i = 0; i < vColors.size(); i++)
			{
				Vec3f Diff = (Vec3f)vColors[i] - Mean3f;

				if (Diff[0] * Diff[0] + Diff[1] * Diff[1] + Diff[2] * Diff[2] < Thresh)
				{
					CurMean3f += vColors[i] * vWeights[i];
					TotalWeight += vWeights[i];
					GroupNum++;
				}
			}

			if (GroupNum == 0)
			{
				break;
			}
			CurMean3f = CurMean3f / TotalWeight;

			Vec3f Diff = CurMean3f - Mean3f;

			if (Diff[0] * Diff[0] + Diff[1] * Diff[1] + Diff[2] * Diff[2] < 10)
			{
				break;
			}

			Mean3f = CurMean3f;

			nIterNum++;
			if (nIterNum > 10)
				break;
		}
	}

	return Mean3f;

}