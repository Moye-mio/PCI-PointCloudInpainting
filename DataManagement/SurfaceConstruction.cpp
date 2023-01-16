#include "pch.h"
#include "SurfaceConstruction.h"
#include "Voxelization.h"
#include "Trimmer.h"
#include "MultiLayerBSplineSurface.h"
#include "HeightMapGenerator.h"
#include "Image.h"

using namespace dataManagement;

bool CSurfaceConstruction::run(const PC_t::Ptr& vCloud, PC_t::Ptr& voResultCloud)
{
	_HIVE_EARLY_RETURN(vCloud == nullptr, "Input Cloud is Nullptr...", false);
	_HIVE_EARLY_RETURN(vCloud->size() == 0, "Input Cloud is Empty...", false);

	{
		std::cout << "Info: Point Cloud Size: " << vCloud->size() << std::endl;
	}

	float Dist = 1.0f;
	int Width = 8;
	int Height = 8;
	int Degree = 3;
	float ScaleInWidth = 1.0f / Width;
	float ScaleInHeight = 1.0f / Height;

	hiveCommon::CCPUTimer Timer;
	double t = Timer.getElapsedTime();
	Timer.start();

	/* Voxelization */
	std::vector<std::pair<Eigen::Vector3i, Point_t>> Voxels;
	core::CVoxelization Voxelization;
	_HIVE_EARLY_RETURN(!Voxelization.setCloud(vCloud), "Error: Voxelization Set Cloud Failed...", false);
	_HIVE_EARLY_RETURN(!Voxelization.generate(Dist), "Error: Voxelization Generate Failed...", false);
	Voxelization.dumpVoxel(Voxels);
	_HIVE_EARLY_RETURN(Voxels.size() == 0, "Error: Voxel Size 0...", false);

	{
		Timer.stop();
		t = Timer.getElapsedTimeInMS();
		std::cout << "Voxel Complete...\nUse time: " << t << "ms" << std::endl;

		__saveVoxel(Voxels);

		t = Timer.getElapsedTime();
		Timer.start();
	}

	/* Trim & Sort */
	Eigen::Matrix<Point_t, -1, -1> SortedVoxels;
	core::CTrimmer Trimmer;
	_HIVE_EARLY_RETURN(!Trimmer.setData(Voxels, Dist), "Error: Trimmer Set Voxels Failed...", false);
	Trimmer.fillAndTrim();
	Trimmer.dumpSorted(SortedVoxels);
	_HIVE_EARLY_RETURN(SortedVoxels.size() == 0, "Error: Sorted Voxels Size 0...", false);

	{
		Timer.stop();
		t = Timer.getElapsedTimeInMS();
		std::cout << "Trim and Sort Complete...\nUse time: " << t << "ms" << std::endl;

		t = Timer.getElapsedTime();
		Timer.start();
	}

	/* Transfer Type */
	Eigen::Matrix<core::SPoint, -1, -1> ControlPoints;
	__transPCLPoints2SPoints(SortedVoxels, ControlPoints);

	{
		Timer.stop();
		t = Timer.getElapsedTimeInMS();
		std::cout << "Transfer Type Complete...\nUse time: " << t << "ms" << std::endl;

		t = Timer.getElapsedTime();
		Timer.start();
	}

	/* B-Spline Surface Construction */
	auto pSurface = std::make_shared<core::CMultiLayerBSplineSurface>(Degree);
	pSurface->setLayer(2);
	pSurface->setMaxSub(5);
	pSurface->setControlPoints(ControlPoints);

	/* Create HeightMap */
	core::CHeightMap HeightMap;
	core::CHeightMapGenerator HGenerator;
	HGenerator.setCloud(vCloud);
	HGenerator.generateBySurface(pSurface, Width, Height);
	HGenerator.dumpHeightMap(HeightMap);
	_ASSERTE(HeightMap.isValid());

	{
		Timer.stop();
		t = Timer.getElapsedTimeInMS();
		std::cout << "B-Spline Surface Construction and Create HeightMap Complete...\nUse time: " << t << "ms" << std::endl;
	}

	{
		__saveHeightMap(HeightMap);
	}

}

void CSurfaceConstruction::__transPCLPoints2SPoints(const Eigen::Matrix<Point_t, -1, -1>& vP, Eigen::Matrix<core::SPoint, -1, -1>& voS)
{
	_ASSERTE(vP.size());

	voS.resize(vP.rows(), vP.cols());
	for (int i = 0; i < vP.rows(); i++)
		for (int k = 0; k < vP.cols(); k++)
			voS.coeffRef(i, k) = __transPCLPoint2SPoint(vP.coeff(i, k));
}

core::SPoint CSurfaceConstruction::__transPCLPoint2SPoint(const Point_t& vP)
{
	return core::SPoint(Eigen::Vector3f(vP.x, vP.y, vP.z));
}

void CSurfaceConstruction::__saveHeightMap(const core::CHeightMap& vMap)
{
	Eigen::Matrix<float, -1, -1> Data;
	Data.resize(vMap.getWidth(), vMap.getHeight());
	for (int i = 0; i < vMap.getWidth(); i++)
		for (int k = 0; k < vMap.getHeight(); k++)
			Data.coeffRef(i, k) = vMap.getValueAt(i, k);

	common::saveImage(Data.cast<int>(), "HeightMap.png");
}

void CSurfaceConstruction::__saveVoxel(const std::vector<std::pair<Eigen::Vector3i, Point_t>>& vVoxels)
{
	PC_t::Ptr pData(new PC_t);
	for (auto& e : vVoxels)
	{
		auto p = e.second;
		pData->push_back(Point_t(p.x, p.y, p.z, 255, 255, 255, 255));
	}
	pcl::io::savePLYFileBinary("Voxel.ply", *pData);
}
