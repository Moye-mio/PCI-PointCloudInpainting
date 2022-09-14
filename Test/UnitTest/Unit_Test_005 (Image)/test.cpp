#include "pch.h"

const std::string LoadPath = "NT_Load.png";
const std::string SavePath = "NT_Save.png";

class TestImage : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}
};

TEST_F(TestImage, NT_Load)
{
	Eigen::Matrix<int, -1, -1> Image;
	common::readImage(LoadPath, Image);

	for (int i = 0; i < Image.rows(); i++)
		for (int k = 0; k < Image.cols(); k++)
			ASSERT_EQ(Image.coeffRef(i, k), (i * Image.cols() + k) * 10);
}

TEST_F(TestImage, NT_Save)
{
	Eigen::Matrix<int, 3, 4> Image;
	for (int i = 0; i < Image.rows(); i++)
		for (int k = 0; k < Image.cols(); k++)
			Image.coeffRef(i, k) = (i * Image.cols() + k) * 10;

	common::saveImage(Image, SavePath);
}