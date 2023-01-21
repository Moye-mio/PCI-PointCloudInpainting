#include "pch.h"

PC_t::Ptr generatePlanePC()
{
	PC_t::Ptr pCloud(new PC_t);
	for (float i = 0; i <= 100; i += 0.1f)
		for (float k = 0; k <= 100; k += 0.1f)
		{
			if (i >= 40 && i <= 60 && k >= 40 && k <= 60)
				continue;
			pCloud->push_back(Point_t(i, k, 1));
		}
	return pCloud;
}

core::SPoint transPoints(const Point_t& vPoint)
{
	core::SPoint Point(vPoint.x, vPoint.y, vPoint.z);
	return Point;
}

TEST(TESTSuface2PCMapper, Plane) 
{
	PC_t::Ptr pCloud = generatePlanePC();

	int Coef = 10;
	Eigen::Matrix<core::SPoint, -1, -1> CPs;
	CPs.resize(10, 10);
	for (int i = 0; i < CPs.rows(); i++)
		for (int k = 0; k < CPs.cols(); k++)
		{
			//if (i <= 5)
			//	CPs.coeffRef(i, k) = core::SPoint(i * Coef, k * Coef, i * Coef);
			//if (i > 5)
			//	CPs.coeffRef(i, k) = core::SPoint(i * Coef, k * Coef, (10 - i) * Coef);
			CPs.coeffRef(i, k) = core::SPoint(i * Coef, k * Coef, 0);
		}

	core::CMultilayerSurface Surface(3);
	Surface.setControlPoints(CPs);
	Surface.setIsSaveMesh(true);
	Surface.preCompute();

	std::vector<std::pair<float, Eigen::Vector2f>> Data;
	for (int i = 0; i < pCloud->size(); i++)
	{
		auto r = Surface.calcProj(transPoints(pCloud->at(i)));

		Data.emplace_back(std::make_pair(r->_Dist, r->_UV));
	}

	core::CHeightMap Map, Mask;
	core::CHeightMapGenerator Generator;
	Generator.generateBySurface(Data, 32, 32);
	Generator.dumpHeightMap(Map);

	/* Save HeightMap and Mask */
	{
		Eigen::Matrix<float, -1, -1> Image;
		Image.resize(Map.getWidth(), Map.getHeight());
		for (int i = 0; i < Map.getWidth(); i++)
			for (int k = 0; k < Map.getHeight(); k++)
				Image.coeffRef(i, k) = Map.getValueAt(i, k);

		common::saveImage(Image.cast<int>(), "HeightMap.png");

		Map.generateMask(Mask);
		Eigen::Matrix<float, -1, -1> MaskImage;
		MaskImage.resize(Mask.getWidth(), Mask.getHeight());
		for (int i = 0; i < Mask.getWidth(); i++)
			for (int k = 0; k < Mask.getHeight(); k++)
				MaskImage.coeffRef(i, k) = Mask.getValueAt(i, k) * 255;

		common::saveImage(MaskImage.cast<int>(), "Mask.png");
	}


	//dataManagement::CSurface2CloudMapper Mapper;
	//Mapper.setSurface();

}