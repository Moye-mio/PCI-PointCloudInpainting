#include "pch.h"
#include "Image.h"

TEST(TestImageInpainting, DT) 
{
	core::CHeightMap Map, Inpainted;

	dataManagement::CImageInpainting Inpainter;
	EXPECT_FALSE(Inpainter.run(Map, Inpainted));
}

TEST(TestImageInpainting, NT_11x11Image1Hole)
{
	core::CHeightMap Map, Inpainted;
	Eigen::Matrix<float, 11, 11> MapData;
	for (int i = 0; i < 11; i++)
		for (int k = 0; k < 11; k++)
		{
			if (i == 5 && k == 5) continue;
			MapData.coeffRef(i, k) = i;
		}
	EXPECT_TRUE(Map.setHeightMap(MapData));
	EXPECT_TRUE(Map.setEmptyAt(5, 5));

	dataManagement::CImageInpainting Inpainter;
	EXPECT_TRUE(Inpainter.run(Map, Inpainted));
	EXPECT_TRUE(Inpainted.getValueAt(5, 5) == 5.0f);
}