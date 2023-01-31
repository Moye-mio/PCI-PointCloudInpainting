#include "pch.h"

#include "ControlPointGenerator.h"

using namespace core;

bool CControlPointGenerator::run(const Eigen::Matrix<Point_t, -1, -1>& vData)
{
	_HIVE_EARLY_RETURN(vData.size() == 0, "Control Points Generator: Data is empty", false);

	Eigen::Matrix<Eigen::Vector3f, -1, -1> Points;
	Points.resize(vData.rows() + 2, vData.cols() + 2);

	int Rows = Points.rows();
	int Cols = Points.cols();

	for (int i = 0; i < vData.rows(); i++)
		for (int k = 0; k < vData.cols(); k++)
			Points.coeffRef(i + 1, k + 1) = Eigen::Vector3f(vData(i, k).x, vData(i, k).y, vData(i, k).z);

	for (int i = 1; i < Rows - 1; i++)
		Points.coeffRef(i, 0) = Points(i, 1) * 2 - Points(i, 2);
	for (int i = 1; i < Cols - 1; i++)
		Points.coeffRef(0, i) = Points(1, i) * 2 - Points(2, i);
	for (int i = 1; i < Rows - 1; i++)
		Points.coeffRef(i, Rows - 1) = Points(i, Rows - 2) * 2 - Points(i, Rows - 3);
	for (int i = 1; i < Cols - 1; i++)
		Points.coeffRef(Cols - 1, i) = Points(Cols - 2, i) * 2 - Points(Cols - 3, i);

	Points.coeffRef(0, 0) = 0.5 * (Points(0, 1) * 2 - Points(0, 2) + Points(1, 0) * 2 - Points(2, 0));
	Points.coeffRef(Rows - 1, 0) = 0.5 * (Points(Rows - 1, 1) * 2 - Points(Rows - 1, 2) + Points(Rows - 2, 0) * 2 - Points(Rows - 3, 0));
	Points.coeffRef(0, Cols - 1) = 0.5 * (Points(1, Cols - 1) * 2 - Points(2, Cols - 1) + Points(0, Cols - 2) * 2 - Points(0, Cols - 3));
	Points.coeffRef(Rows - 1, Cols - 1) = 0.5 * (Points(Rows - 2, Cols - 1) * 2 - Points(Rows - 3, Cols - 1) + Points(Rows - 1, Cols - 2) * 2 - Points(Rows - 1, Cols - 3));

	m_Points.resize(Rows, Cols);
	for (int i = 0; i < Rows; i++)
		for (int k = 0; k < Cols; k++)
			m_Points.coeffRef(i, k) = Point_t(Points(i, k)[0], Points(i, k)[1], Points(i, k)[2]);

	return true;
}
