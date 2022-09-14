#pragma once

#include <Eigen/Eigen>

namespace common
{
	bool readImage(const std::string& vPath, Eigen::Matrix<int, -1, -1>& voImage);

	void saveImage(const Eigen::Matrix<int, -1, -1>& vImage, const std::string& vPath);
}

