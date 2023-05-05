#include "pch.h"

#include "MultilayerSurface.h"
#include "HeightMapGenerator.h"
#include "Image.h"

#include "SurfaceUVGenerator.h"
#include "VertexSampler.h"
#include "NormalSampler.h"
#include "DistSampler.h"
#include "Surface2PCMapper.h"

#include <pcl/filters/voxel_grid.h>


#include <opencv2/opencv.hpp>


const std::string Path1 = TESTMODEL_DIR + std::string("/Trimmed/Walkway/WH_Walkway_Trim.ply");
const std::string Path2 = TESTMODEL_DIR + std::string("/Trimmed/Walkway/GT_Walkway_Trim.ply");
const std::string Path3 = TESTMODEL_DIR + std::string("/Trimmed/Terrain/Sample_0_120_Seg_Sub_6_ASCII.ply");
const std::string Path4 = TESTMODEL_DIR + std::string("/Trimmed/Terrain/Sample_0_120_Seg.ply");
const std::string Path5 = TESTMODEL_DIR + std::string("/ExperimentResult/DetectHole/Complete.ply");
const std::string Path6 = TESTMODEL_DIR + std::string("/ExperimentResult/DetectHole/Hole.ply");
const std::string Path7 = TESTMODEL_DIR + std::string("/Trimmed/Terrain/ASCII/Sample_0_120_Seg_Sub_6_ASCII.ply");

PC_t::Ptr loadPC(const std::string& vPath)
{
	std::string FileName = hiveUtility::hiveLocateFile(vPath);
	_ASSERTE(!FileName.empty());

	PC_t::Ptr pCloud(new PC_t);
	int r = pcl::io::loadPLYFile<Point_t>(FileName, *pCloud);
	_ASSERTE(r != -1);
	_ASSERTE(pCloud->size());
	std::cout << "Model Point Size: " << pCloud->size() << std::endl;

	return pCloud;
}

void projPoints(const std::shared_ptr<pcl::on_nurbs::FittingSurface>& vFit, core::CMultilayerSurface* vSurface, const PC_t::Ptr& vCloud, std::vector<std::pair<float, Eigen::Vector2f>>& voData)
{
	for (int i = 0; i < vCloud->size(); i++)
	{
		const auto& PCLPoint = vCloud->at(i);
		auto r = vSurface->calcProj(core::SPoint(PCLPoint.x, PCLPoint.y, PCLPoint.z));
		if (r.has_value() == false)
		{
			hiveEventLogger::hiveOutputEvent(_FORMAT_STR4("Point Number [%1%], Pos [%2%, %3%, %4%] calc proj info failed", i, PCLPoint.x, PCLPoint.y, PCLPoint.z));
			continue;
		}
		Eigen::Vector2d Hint((double)r->_UV[0], (double)r->_UV[1]);

		Eigen::Vector3d Point(vCloud->at(i).x, vCloud->at(i).y, vCloud->at(i).z);
		Eigen::Vector3d p;
		Eigen::Vector2d UV = vFit->inverseMapping(vFit->m_nurbs, Point, Hint, p, 100, 1e-6, true);
		if (UV[0] < 0 || UV[0] > 1 || UV[1] < 0 || UV[1] > 1 || std::isnan(p.x()) || std::isnan(p.y()) || std::isnan(p.z()))
		{
			hiveEventLogger::hiveOutputEvent(_FORMAT_STR6("Point Number [%1%], UV [%2%, %3%], Pos [%4%, %5%, %6%]", i, UV[0], UV[1], p[0], p[1], p[2]));
			continue;
		}
		const auto Sample = vFit->m_nurbs.PointAt(UV[0], UV[1]);
		/*hiveEventLogger::hiveOutputEvent(_FORMAT_STR6("[%1%, %2%, %3%] -> [%4%, %5%, %6%]", p.x(), p.y(), p.z(), Sample.x, Sample.y, Sample.z));
		if ((Eigen::Vector3d(Sample.x, Sample.y, Sample.z) - p).norm() > 0.00001)
			hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("Diff too big [%1%]", (Eigen::Vector3d(Sample.x, Sample.y, Sample.z) - p).norm()));*/

		p = Eigen::Vector3d(Sample.x, Sample.y, Sample.z);
		float Dist = (Point - p).norm();
		voData.emplace_back(std::make_pair(Dist, UV.cast<float>()));
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR8("[%1%, %2%, %3%] -> [%4%, %5%, %6%], UV: [%7%, %8%]", Point[0], Point[1], Point[2], p[0], p[1], p[2], UV[0], UV[1]));
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("Dist: [%1%]", Dist));
	}
	hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Point Cloud [%1%] convert 2 Data [%2%]", vCloud->size(), voData.size()));
}

void saveMap2Image(const core::CHeightMap& vMap, const std::string& vPath, int vCoef = 1)
{
	Eigen::Matrix<float, -1, -1> Image;
	Image.resize(vMap.getWidth(), vMap.getHeight());
	for (int i = 0; i < vMap.getWidth(); i++)
		for (int k = 0; k < vMap.getHeight(); k++)
			Image.coeffRef(i, k) = vMap.getValueAt(i, k) * vCoef;

	common::saveImage(Image.cast<int>(), vPath);
}

void tuneMask(core::CHeightMap& vioMask)
{
	int Tune = 3;
	for (int i = 0; i < vioMask.getWidth(); i++)
		for (int k = 0; k < vioMask.getHeight(); k++)
		{
			if (i < Tune || k < Tune || i > vioMask.getWidth() - Tune - 1 || k > vioMask.getHeight() - Tune - 1)
			{
				vioMask.setValueAt(0, i, k);
			}
		}

	const core::CHeightMap Map = vioMask;

	for (int i = 1; i < vioMask.getWidth() - 1; i++)
		for (int k = 1; k < vioMask.getHeight() - 1; k++)
		{
			if (Map.getValueAt(i - 1, k) == 1 || Map.getValueAt(i, k - 1) == 1 || Map.getValueAt(i + 1, k) == 1 || Map.getValueAt(i, k + 1) == 1)
				vioMask.setValueAt(1, i, k);
		}
}

bool map2Cloud(const core::CHeightMap& vRaw, const core::CHeightMap& vMask, const core::CHeightMap& vInpainted, int vSPP, const std::shared_ptr<pcl::on_nurbs::FittingSurface>& vFit, const std::shared_ptr<core::CMultilayerSurface>& vSurface, PC_t::Ptr& vCloud)
{
	core::CHeightMap Mask = vMask;
	if (!vMask.isValid())
		vRaw.generateMask(Mask);

	_HIVE_EARLY_RETURN(Mask.isValid() == false, "ERROR: Mask Image is not Valid", false);

	std::vector<Eigen::Vector2f> UVs;
	std::vector<Eigen::Vector3f> Normals;
	std::vector<float> Dists;
	std::vector<core::SVertex> Samples;
	Eigen::Matrix<core::SVertex, -1, -1> VertexData;
	vSurface->dumpLatestLayer(VertexData);
	hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("dump Vertex Data [%1% x %2%] dims from Refined Surface", VertexData.rows(), VertexData.cols()));
	_HIVE_EARLY_RETURN(VertexData.size() == 0, "ERROR: VertexData is Empty", false);

	/* Generate UV */
	core::CSurfaceUVGenerator Generator;
	if (Generator.generateUVSamples(Mask, vSPP) == false) return false;
	Generator.dumpSamples(UVs);
	_HIVE_EARLY_RETURN(UVs.size() == 0, "ERROR: UV Vector is Empty", false);

	/* Sample Vertex */
	for (const auto& e : UVs)
	{
		const auto Sample = vFit->m_nurbs.PointAt(e[0], e[1]);
		if (std::isnan(Sample.x) || std::isnan(Sample.y) || std::isnan(Sample.z))
			hiveEventLogger::hiveOutputEvent(_FORMAT_STR5("nan: UV: [%1%, %2%], Value: [%3%, %4%, %5%]", e[0], e[1], Sample.x, Sample.y, Sample.z));
		Samples.emplace_back(core::SVertex(Sample.x, Sample.y, Sample.z));
	}
	hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("Sample Vector size: %1%", Samples.size()));

	/* Sample Normal */
	core::CNormalSampler NormalSampler;
	if (NormalSampler.setData(VertexData) == false) return false;
	for (const auto& e : UVs)
	{
		auto r = NormalSampler.sample(e);
		_HIVE_EARLY_RETURN(r.has_value() == false, "ERROR: Surface 2 Cloud Mapper: Normal Sampler Failed", false);
		Normals.emplace_back(r.value());
	}

	/* Sample Dist */
	core::CDistSampler DistSampler;
	if (DistSampler.setResource(vInpainted) == false) return false;
	if (DistSampler.samplebyUV(UVs, Dists) == false) return false;

	_HIVE_EARLY_RETURN(Normals.size() != Dists.size() || Dists.size() != Samples.size(), "ERROR: Surface 2 Cloud Mapper: Normals, Dists, UVs Size is not Same", false);
	hiveEventLogger::hiveOutputEvent("Start Mapping: ");

	/* Map */
	core::CSurface2PCMapper Mapper;
	for (int i = 0; i < Normals.size(); i++)
	{
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR8("Point [%1%]: Start Point(%2%, %3%, %4%), Normal(%5%, %6%, %7%), Dist %8%", i, Samples[i].x, Samples[i].y, Samples[i].z, Normals[i][0], Normals[i][1], Normals[i][2], Dists[i]));

		{
			if (Samples[i].u < 0.5f)
				Normals[i] *= -1;
		}

		//if (std::isnan(Dists[i]) || std::fabsf(Dists[i]) > 100) continue;
		Dists[i] = 0;

		auto r = Mapper.generatePoint(Samples[i], Normals[i], Dists[i]);

		_HIVE_EARLY_RETURN(r.has_value() == false, "ERROR: Surface 2 Cloud Mapper: Mapper generate Point Failed", false);
		vCloud->emplace_back(r.value());
	}

	hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("New Cloud Size [%1%]", vCloud->size()));

	return true;
}

Eigen::Vector2i __computeOffset(const Eigen::Vector2f& vUV, int vWidth, int vHeight)
{
	Eigen::Vector2f Scale(1.0f / vWidth, 1.0f / vHeight);

	int OffsetX = vUV[0] / Scale[0];
	int OffsetY = vUV[1] / Scale[1];

	if (OffsetX == vWidth) OffsetX--;
	if (OffsetY == vHeight) OffsetY--;

	return Eigen::Vector2i(OffsetX, OffsetY);
}

//TEST(NurbsFitting, DT) 
//{
//	PC_t::Ptr pCloud;
//	core::CNurbsFitting Fitting;
//	EXPECT_FALSE(Fitting.run(pCloud, 3, 4, 10));
//	pCloud = loadPC(Path1);
//	EXPECT_FALSE(Fitting.run(pCloud, -1, 4, 10));
//	EXPECT_FALSE(Fitting.run(pCloud, 3, -1, 10));
//	EXPECT_FALSE(Fitting.run(pCloud, 3, 4, -1));
//}


TEST(NurbsFitting, NT_CrossPlane)
{
	PC_t::Ptr pCloudWH = loadPC(Path6);

	//pcl::VoxelGrid<Point_t> Sor;
	//Sor.filter(*pCloudWH);

	{
		/*for (auto& e : *pCloudWH)
		{
			float Temp = e.y;
			e.y = e.z;
			e.z = Temp;
		}*/
	}

	//	Eigen::Vector3f Min(FLT_MAX, FLT_MAX, FLT_MAX);
	//	for (auto& e : *pCloudWH)
	//	{
	//		Min.x() = (Min.x() > e.x) ? e.x : Min.x();
	//		Min.y() = (Min.y() > e.y) ? e.y : Min.y();
	//		Min.z() = (Min.z() > e.z) ? e.z : Min.z();
	//	}
	//	for (auto& e : *pCloudWH)
	//	{
	//		e.x -= Min.x();
	//		e.y -= Min.y();
	//		e.z -= Min.z();
	//	}
	//}

	//pcl::io::savePLYFileBinary("Mesh/Cloud.ply", *pCloudWH);


	//PC_t::Ptr pCloudGT = loadPC(Path4);
	core::CNurbsFitting Fitting;
	std::shared_ptr<pcl::on_nurbs::FittingSurface> Fit;
	EXPECT_TRUE(Fitting.run(pCloudWH, 3, 5, 10));	/* 19 x 19 */

	//Fitting.dumpFittingSurface(Nurbs);
	Fitting.dumpFitting(Fit);
	ON_NurbsSurface Nurbs = Fit->m_nurbs;

	Eigen::Matrix<core::SPoint, -1, -1> Ctrlpts;
	Ctrlpts.resize(Nurbs.m_cv_count[0], Nurbs.m_cv_count[1]);
	for (int i = 0; i < Ctrlpts.rows(); i++)
		for (int k = 0; k < Ctrlpts.cols(); k++)
		{
			ON_3dPoint p;
			Nurbs.GetCV(i, k, p);
			Ctrlpts.coeffRef(i, k) = core::SPoint(p.x, p.y, p.z);
		}

	// save Ctrlpts to file
	{
		std::ofstream OutFile("B-spline surface/Ctrlpts_WH.txt");
		std::ofstream OutFile2("B-spline surface/Knot_WH.txt");
		for (int i = 0; i < Ctrlpts.rows(); i++)
		{
			std::string StrX = "";
			std::string StrY = "";
			std::string StrZ = "";
			for (int k = 0; k < Ctrlpts.cols(); k++)
			{
				const auto p = Ctrlpts.coeff(i, k);
				StrX += std::to_string(p.x()) + " ";
				StrY += std::to_string(p.y()) + " ";
				StrZ += std::to_string(p.z()) + " ";
			}

			OutFile << StrX << std::endl << StrY << std::endl << StrZ << std::endl << std::endl;
		}

		{
			std::string StrU = "";
			std::string StrV = "";

			for (int i = 0; i < Nurbs.m_order[0] + Ctrlpts.rows() - 2; i++)
			{
				StrU += std::to_string(Nurbs.m_knot[0][i]) + " ";
				StrV += std::to_string(Nurbs.m_knot[1][i]) + " ";
			}

			OutFile2 << StrU << std::endl << StrV << std::endl;
		}

		OutFile.close();
		OutFile2.close();
	}

	core::CMultilayerSurface* pSurface(new core::CMultilayerSurface(3));
	pSurface->setControlPoints(Ctrlpts);
	pSurface->setIsSaveMesh(true, "Mesh/WantedArea.obj");
	pSurface->setSubNumber(10);
	pSurface->setSubLayer(2);
	EXPECT_TRUE(pSurface->preCompute());

	std::vector<std::pair<float, Eigen::Vector2f>> DataWH;
	projPoints(Fit, pSurface, pCloudWH, DataWH);

	int Res = 64;
	{
		std::vector<int> Indices;
		
		// 17, 9; 32, 16
		int Count = 0;
		for (const auto& e : DataWH)
		{
			Eigen::Vector2i Offset = __computeOffset(e.second, Res, Res);
			if (Offset[0] >= 16 && Offset[0] <= 33 && Offset[1] >= 8 && Offset[1] <= 17)
				Indices.emplace_back(Count);
			Count++;
		}

		std::ofstream OutFile("Mesh/Proj.txt");
		for (int i = 0; i < Indices.size(); i++)
		{
			auto [Dist, Pos] = DataWH[Indices[i]];
			OutFile << std::to_string(Pos[0]) << " " << std::to_string(Pos[1]) << std::endl;
		}
		OutFile.close();
	}


	core::CHeightMapGenerator Generator;
	core::CHeightMap Map, Mask;
	Generator.generateBySurface(DataWH, Res, Res);
	Generator.dumpHeightMap(Map);
	Map.generateMask(Mask);
	saveMap2Image(Mask, "Mesh/Mask.png", 255);

	for (int i = 0; i < Res; i++)
		for (int k = 0; k < Res; k++)
		{
			float Value = Mask.getValueAt(i, k);
			if (Value != 0 && (i < 5 || k < 5 || i > 58 || k > 58)) Mask.setValueAt(0, i, k);
		}

	Eigen::Vector2i Min(Res, Res);
	Eigen::Vector2i Max(0, 0);

	for (int i = 0; i < Res; i++)
		for (int k = 0; k < Res; k++)
		{
			if (Mask.getValueAt(i, k) != 0)
			{
				Min.x() = (Min.x() > i) ? i : Min.x();
				Max.x() = (Max.x() < i) ? i : Max.x();
				Min.y() = (Min.y() > k) ? k : Min.y();
				Max.y() = (Max.y() < k) ? k : Max.y();
			}
		}

	cv::Mat Image(Max[0] - Min[0] + 1, Max[1] - Min[1] + 1, CV_8UC3);
	for (int i = Min[0]; i <= Max[0]; i++)
		for (int k = Min[1]; k <= Max[1]; k++)
		{
			if (Map.getValueAt(i, k) == 0)
				Image.at<cv::Vec3b>(i - Min[0], k - Min[1]) = cv::Vec3b(76, 76, 76);
			else
				Image.at<cv::Vec3b>(i - Min[0], k - Min[1]) = cv::Vec3b(255, 127, 126);
		}

	cv::imwrite("Mesh/cvMask.png", Image);
	/*std::vector<std::pair<float, Eigen::Vector2f>> DataWH, DataGT;
	projPoints(Fit, pSurface, pCloudWH, DataWH);
	projPoints(Fit, pSurface, pCloudGT, DataGT);

	core::CHeightMapGenerator MapGeneraterWH, MapGeneraterGT;
	core::CHeightMap Map, Mask, Inpainted, InpaintedMask;
	int Width = 32;
	int Height = 32;
	EXPECT_TRUE(MapGeneraterWH.generateBySurface(DataWH, Width, Height));
	MapGeneraterWH.dumpHeightMap(Map);
	saveMap2Image(Map, "HeightMap.png", 100);
	Map.generateMask(Mask);
	saveMap2Image(Mask, "MaskRaw.png", 255);
	tuneMask(Mask);
	saveMap2Image(Mask, "MaskTune.png", 255);

	EXPECT_TRUE(MapGeneraterGT.generateBySurface(DataGT, Width, Height));
	MapGeneraterGT.dumpHeightMap(Inpainted);
	saveMap2Image(Inpainted, "InpaintedMap.png", 100);
	Inpainted.generateMask(InpaintedMask);
	saveMap2Image(InpaintedMask, "InpaintedMask.png", 255);

	int SPP = 100;
	std::shared_ptr<core::CMultilayerSurface> pTrSurface(pSurface);
	PC_t::Ptr pCloud(new PC_t);
	EXPECT_TRUE(map2Cloud(Map, Mask, Inpainted, SPP, Fit, pTrSurface, pCloud));

	for (auto& e : *pCloud)
	{
		e.r = 253;
		e.g = 199;
		e.b = 83;
	}

	for (auto& e : *pCloudWH)
	{
		e.r = 255;
		e.g = 255;
		e.b = 255;
		pCloud->emplace_back(e);
	}

	pcl::io::savePLYFileBinary("newCloud.ply", *pCloud);*/
}


