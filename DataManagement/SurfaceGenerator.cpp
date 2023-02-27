#include "pch.h"
#include "SurfaceGenerator.h"
#include "PCLoader.h"
#include "BiasedVoxelization.h"
#include "Sorter.h"
#include "ImageInpainting.h"
#include "ControlPointGenerator.h"

using namespace dataManagement;

bool CSurfaceGenerator::run(const std::string& vPath)
{
	/* Load Point Cloud */
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<dataManagement::IPCLoader>(hiveUtility::hiveGetFileSuffix(vPath));
	_HIVE_EARLY_RETURN(pTileLoader == nullptr, "Surface Generator: Loader failed to create", false);
	PC_t::Ptr pData = pTileLoader->loadDataFromFile(vPath);
	_HIVE_EARLY_RETURN(pData->size() == 0, "Surface Generator: Point Cloud is empty", false);

	/* Voxelization */
	int VoxelThres = 5;
	float Dist = 0.2f;
	std::vector<std::pair<Eigen::Vector3i, Point_t>> Voxels;
	core::CBiasedVoxelization Voxelization;
	_HIVE_EARLY_RETURN(Voxelization.setCloud(pData) == false, "Surface Generator: Voxel data is empty", false);
	_HIVE_EARLY_RETURN(Voxelization.setDenoiseThres(VoxelThres) == false, "Surface Generator: Voxel Thres is Invalid", false);
	_HIVE_EARLY_RETURN(Voxelization.generate(Dist) == false, "Surface Generator: Voxel generate failed", false);
	Voxelization.dumpVoxel(Voxels);
	_HIVE_EARLY_RETURN(Voxels.size() == 0, "Surface Generator: Voxel list is empty", false);

	/* Sort Voxels */
	Eigen::Matrix<Point_t, -1, -1> Sorted, ControlPoints, Sorted2;
	core::CSorter Sorter;
	_HIVE_EARLY_RETURN(Sorter.sort(Voxels) == false, "Surface Generator: Sort failed", false);
	Sorter.dumpSortedData(Sorted);
	_HIVE_EARLY_RETURN(Sorted.size() == 0, "Surface Generator: Sorted is empty", false);

	{
		Sorted2.resize(Sorted.rows() - 2, Sorted.cols() - 2);
		for (int i = 0; i < Sorted2.rows(); i++)
			for (int k = 0; k < Sorted2.cols(); k++)
				Sorted2.coeffRef(i, k) = Sorted.coeff(i + 1, k + 1);

		core::CControlPointGenerator G1;
		G1.run(Sorted2);
		G1.dumpControlPoints(Sorted2);

		for (int i = 0; i < Sorted2.rows(); i++)
			for (int k = 0; k < Sorted.cols(); k++)
				if (__isPointSame(Sorted2.coeff(i, k), Point_t(-FLT_MAX, -FLT_MAX, -FLT_MAX)))
				{
					const auto& Point = Sorted2.coeff(i, k);
					std::vector<Point_t> Points;
					Points.emplace_back(Sorted2.coeff(i - 1, k));
					Points.emplace_back(Sorted2.coeff(i, k - 1));
					Points.emplace_back(Sorted2.coeff(i + 1, k));
					Points.emplace_back(Sorted2.coeff(i, k + 1));

					Point_t p(0, 0, 0);
					for (const auto& e : Points)
					{
						p.x += e.x;
						p.y += e.y;
						p.z += e.z;
					}

					p.x /= Points.size();
					p.y /= Points.size();
					p.z /= Points.size();

					Sorted2.coeffRef(i, k) = p;
					
					hiveEventLogger::hiveOutputEvent(_FORMAT_STR5("Control Points: [%1%, %2%], Value: [%3%, %4%, %5%]", i, k, p.x, p.y, p.z));
				}
	}

	/* Generate Control Points */
	core::CControlPointGenerator Generator;
	_HIVE_EARLY_RETURN(Generator.run(Sorted2) == false, "Surface Generator: Control Point Generator failed", false);
	Generator.dumpControlPoints(ControlPoints);

	{
		PC_t::Ptr pCloud(new PC_t);
		for (int i = 0; i < ControlPoints.rows(); i++)
			for (int k = 0; k < ControlPoints.cols(); k++)
				pCloud->emplace_back(ControlPoints.coeff(i, k));
		pcl::io::savePLYFileBinary("Voxel.ply", *pCloud);
	}

	m_ControlPoints = ControlPoints;

	/*  */

	/* Inpaint Voxels */
	/*bool IsToInpainted = __isEmptyInVoxels(Sorted);
	if (IsToInpainted)
	{

	}*/

	Eigen::Matrix<core::SPoint, -1, -1> SPoints;
	__castPCLPoint2SPoint(m_ControlPoints, SPoints);
	
	/* generate Surface */
	core::CMultilayerSurface* pSurface(new core::CMultilayerSurface(3));
	pSurface->setControlPoints(SPoints);
	pSurface->setSubLayer(3);
	pSurface->setIsSaveMesh(true, "Surface.obj");
	_HIVE_EARLY_RETURN(pSurface->preCompute() == false, "Surface Generator: Surface precompute failed", false);

	std::vector<std::pair<float, Eigen::Vector2f>> Data;
	__extractProjData(pSurface, pData, Data);
	m_pSurface.reset(pSurface);
	m_Data = Data;

	return true;
}

bool CSurfaceGenerator::__isEmptyInVoxels(const Eigen::Matrix<Point_t, -1, -1>& vVoxels)
{
	for (int i = 0; i < vVoxels.rows(); i++)
		for (int k = 0; k < vVoxels.cols(); k++)
			if (__isPointSame(vVoxels.coeff(i, k), Point_t(-FLT_MAX, -FLT_MAX, -FLT_MAX)))
				return true;
	return false;
}

bool CSurfaceGenerator::__isPointSame(const Point_t& vLhs, const Point_t& vRhs)
{
	if (vLhs.x == vRhs.x && vLhs.y == vRhs.y && vLhs.z == vRhs.z && vLhs.r == vRhs.r && vLhs.g == vRhs.g && vLhs.b == vRhs.b && vLhs.a == vRhs.a)
		return true;
	else
		return false;
}

void CSurfaceGenerator::__castPCLPoint2SPoint(const Eigen::Matrix<Point_t, -1, -1>& vPCLPoints, Eigen::Matrix<core::SPoint, -1, -1>& voSPoints)
{
	voSPoints.resize(vPCLPoints.rows(), vPCLPoints.cols());
	for (int i = 0; i < vPCLPoints.rows(); i++)
		for (int k = 0; k < vPCLPoints.cols(); k++)
		{
			const auto& Point = vPCLPoints.coeff(i, k);
			voSPoints.coeffRef(i, k) = core::SPoint(Point.x, Point.y, Point.z);
		}
}

void CSurfaceGenerator::__extractProjData(core::CMultilayerSurface* vSurface, const PC_t::Ptr& vCloud, std::vector<std::pair<float, Eigen::Vector2f>>& voData)
{
	voData.clear();
	std::cout << "Proj Data: " << std::endl;

	for (int i = 0; i < vCloud->size(); i++)
	{
		Point_t Point = vCloud->at(i);
		auto r = vSurface->calcProj(core::SPoint(Point.x, Point.y, Point.z));
		if (r.has_value() == false) continue;
		voData.emplace_back(std::make_pair(r->_Dist, r->_UV));
	}
}