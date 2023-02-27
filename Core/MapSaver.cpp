#include "pch.h"
#include "MapSaver.h"
#include "Image.h"

using namespace core;

bool CMapSaver::save(const core::CHeightMap& vMap, const std::string& vPath, int vCoef)
{
	_HIVE_EARLY_RETURN(vMap.isValid() == false, "Height Map is invalid", false);

	Eigen::Matrix<float, -1, -1> Data;
	Data.resize(vMap.getWidth(), vMap.getHeight());
	for (int i = 0; i < Data.rows(); i++)
		for (int k = 0; k < Data.cols(); k++)
			Data.coeffRef(i, k) = vMap.getValueAt(i, k) * vCoef;

	common::saveImage(Data.cast<int>(), vPath);
	return true;
}

bool CMapSaver::save(const core::CGradientMap& vMap, const std::string& vFileName, int vCoef)
{
	_HIVE_EARLY_RETURN(vMap.isValid() == false, "Gradient Map is invalid", false);

	Eigen::Matrix<float, -1, -1> DataX, DataY;
	DataX.resize(vMap.getWidth(), vMap.getHeight());
	DataY.resize(vMap.getWidth(), vMap.getHeight());
	for (int i = 0; i < DataX.rows(); i++)
		for (int k = 0; k < DataX.cols(); k++)
		{
			DataX.coeffRef(i, k) = vMap.getValueAt(i, k)[0] * vCoef;
			DataY.coeffRef(i, k) = vMap.getValueAt(i, k)[1] * vCoef;
		}

	common::saveImage(DataX.cast<int>(), vFileName + "_X.png");
	common::saveImage(DataY.cast<int>(), vFileName + "_Y.png");
	return true;
}
