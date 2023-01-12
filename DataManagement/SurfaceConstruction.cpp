#include "pch.h"
#include "SurfaceConstruction.h"
#include "Voxelization.h"
#include "Trimmer.h"
#include "MultiLayerBSplineSurface.h"
#include "HeightMapGenerator.h"

using namespace dataManagement;

void CSurfaceConstruction::run(const PC_t::Ptr& vCloud, PC_t::Ptr& voResultCloud)
{
	_ASSERTE(vCloud != nullptr);
	_ASSERTE(vCloud->size());

	float Dist = 0.9f;
	int Width = 128;
	int Height = 128;
	float ScaleInWidth = 1.0f / Width;
	float ScaleInHeight = 1.0f / Height;

	/* Voxelization */
	std::vector<std::pair<Eigen::Vector3i, Point_t>> Voxels;
	core::CVoxelization Voxelization;
	Voxelization.setCloud(vCloud);
	Voxelization.generate(Dist);
	Voxelization.dumpVoxel(Voxels);
	_ASSERTE(Voxels.size());

	{
		PC_t::Ptr pData(new PC_t);
		for (auto& e : Voxels)
		{
			auto p = e.second;
			pData->push_back(Point_t(p.x, p.y, p.z, 255, 255, 255, 255));
		}
		pcl::io::savePLYFileBinary("Voxel.ply", *pData);
	}

	/* Trim & Sort */
	Eigen::Matrix<Point_t, -1, -1> SortedVoxels;
	core::CTrimmer Trimmer;
	Trimmer.setData(Voxels, Dist);
	Trimmer.fillAndTrim();
	Trimmer.dumpSorted(SortedVoxels);
	_ASSERTE(SortedVoxels.size());

	/* Transfer Type */
	Eigen::Matrix<core::SPoint, -1, -1> ControlPoints;
	__transPCLPoints2SPoints(SortedVoxels, ControlPoints);

	/* B-Spline Surface Construction */
	core::CMultiLayerBSplineSurface Surface(3);
	Surface.setLayer(2);
	Surface.setMaxSub(8);
	Surface.setControlPoints(ControlPoints);
	
	/* Create HeightMap */
	core::CHeightMap HeightMap, HeightMapInpainted;
	core::CHeightMapGenerator HGenerator;
	HGenerator.setCloud(vCloud);
	HGenerator.generate(128, 128);			/* Magic Number */
	HGenerator.dumpHeightMap(HeightMap);
	_ASSERTE(HeightMap.isValid());



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
