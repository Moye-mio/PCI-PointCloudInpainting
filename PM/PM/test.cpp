#include <opencv2/opencv.hpp>
#include <iostream>

int main()
{
	cv::Mat Image;
	Image = cv::imread("./images/forest.bmp");
	cv::imshow("", Image);
	cv::waitKey(0);

	return 0;
}
