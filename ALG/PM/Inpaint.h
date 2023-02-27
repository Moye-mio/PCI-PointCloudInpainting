#pragma once

namespace PM
{
	class CInpainter
	{
	public:
		CInpainter(const cv::Mat& vRaw, const cv::Mat& vMask, int vPatchSize = 7, int vPyramid = 3, int vMaxIterNumber = 15, float vThreshold = 10.0f);
		~CInpainter() {}

		void inpaint();
		void inpaint(cv::VideoWriter& voWritter);
		void dumpResult(cv::Mat& voResult) { voResult = m_Inpainted; }

	private:
		void __inpaint(bool vIsWriteVideo, cv::VideoWriter& voWritter);
		void __inpaintC1(bool vIsWriteVideo, cv::VideoWriter& voWritter);
		void __inpaintC2(bool vIsWriteVideo, cv::VideoWriter& voWritter);
		Eigen::Vector2f __computeRange(const cv::Mat& vMat, const cv::Mat& vMask);


	private:
		cv::Mat m_Raw;			/* CV_32F */
		cv::Mat m_Mask;			/* CV_8UC1 */
		cv::Mat m_Inpainted;	/* CV_32F */
		int m_PatchSize;
		int m_Pyramid;
		int m_MaxIterNumber;
		float m_Threshold;
	};
}