#include "Image.h"

#include <opencv2/opencv.hpp>

#include <boost/format.hpp>
#include "common/EventLoggerInterface.h"
#include "common/DesignPatternInterface.h"
#include "common/UtilityInterface.h"

using namespace common;

bool common::readImage(const std::string& vPath, Eigen::Matrix<int, -1, -1>& voImage)
{
	_ASSERTE(!vPath.empty());

	std::string FileName = hiveUtility::hiveLocateFile(vPath);
	_HIVE_EARLY_RETURN(FileName.empty(), _FORMAT_STR1("Fail to load file [%1%] because it does not exist.", vPath), false);

	cv::Mat Image = cv::imread(vPath, cv::IMREAD_GRAYSCALE);
	voImage = Eigen::MatrixXi::Constant(Image.rows, Image.cols, 0);
	for (int i = 0; i < Image.rows; i++)
		for (int k = 0; k < Image.cols; k++)
			voImage.coeffRef(i, k) = static_cast<std::uint32_t>(Image.at<unsigned char>(i, k));

	return true;

}

void common::saveImage(const Eigen::Matrix<int, -1, -1>& vImage, const std::string& vPath)
{
	_ASSERTE(vImage.size());
	cv::Mat Image(vImage.rows(), vImage.cols(), CV_8UC1);
	for (int i = 0; i < vImage.rows(); i++)
		for (int k = 0; k < vImage.cols(); k++)
			Image.at<unsigned char>(i, k) = vImage(i, k);

	cv::imwrite(vPath, Image);
}