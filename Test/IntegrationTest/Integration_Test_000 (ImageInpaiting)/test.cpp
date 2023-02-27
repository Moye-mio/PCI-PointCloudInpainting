#include "pch.h"
#include "Image.h"

core::CHeightMap castCVMat2HeightMap(const cv::Mat& vSrc)
{
	Eigen::Matrix<float, -1, -1> Data;
	Data.resize(vSrc.rows, vSrc.cols);
	for (int i = 0; i < Data.rows(); i++)
		for (int k = 0; k < Data.cols(); k++)
			Data.coeffRef(i, k) = (float)vSrc.at<cv::Vec3b>(i, k)[0];
	
	core::CHeightMap Map;
	Map.setHeightMap(Data);
	return Map;
}

void setEmpty(core::CHeightMap& vMap, const core::CHeightMap& vMask)
{
	for (int i = 0; i < vMap.getWidth(); i++)
		for (int k = 0; k < vMap.getHeight(); k++)
			if (vMask.getValueAt(i, k) == 1)
				vMap.setEmptyAt(i, k);
}

TEST(TestImageInpainting, DT) 
{
	core::CHeightMap Map, Inpainted;

	dataManagement::CImageInpainting Inpainter;
	EXPECT_FALSE(Inpainter.run(Map, Inpainted));
}

TEST(TestImageInpainting, NT_ReadImageFromDisk)
{
	cv::Mat HeightMap = cv::imread("Image/HeightMap.png");
	cv::Mat MaskMap = cv::imread("Image/Mask.png");

	core::CHeightMap Map = castCVMat2HeightMap(HeightMap);
	core::CHeightMap Mask = castCVMat2HeightMap(MaskMap);
	setEmpty(Map, Mask);
	core::CHeightMap Inpainted;

	dataManagement::CImageInpainting Inpainter;
	EXPECT_FALSE(Inpainter.run(Map, Inpainted));
}

TEST(TestImageInpainting, NT_11x11Image1Hole)
{
	float Epsilon = 0.0001f;
	core::CHeightMap Map, Inpainted;
	Eigen::Matrix<float, 11, 11> MapData;
	for (int i = 0; i < 11; i++)
		for (int k = 0; k < 11; k++)
			MapData.coeffRef(i, k) = i;

	EXPECT_TRUE(Map.setHeightMap(MapData));
	EXPECT_TRUE(Map.setEmptyAt(5, 5));

	dataManagement::CImageInpainting Inpainter;
	EXPECT_TRUE(Inpainter.run(Map, Inpainted));
}

TEST(TestImageInpainting, NT_101x101Image1Hole)
{
	float Epsilon = 0.0001f;
	core::CHeightMap Map, Inpainted;
	Eigen::Matrix<float, 101, 101> MapData;
	for (int i = 0; i < 101; i++)
		for (int k = 0; k < 101; k++)
			MapData.coeffRef(i, k) = i;
		
	EXPECT_TRUE(Map.setHeightMap(MapData));
	for (int i = 45; i < 55; i++)
		for (int k = 45; k < 55; k++)
			EXPECT_TRUE(Map.setEmptyAt(i, k));

	dataManagement::CImageInpainting Inpainter;
	EXPECT_TRUE(Inpainter.run(Map, Inpainted));
	for (int i = 45; i < 55; i++)
		for (int k = 45; k < 55; k++)
			EXPECT_TRUE(std::fabsf(Inpainted.getValueAt(i, k) - MapData.coeff(i, k)) < Epsilon);
}