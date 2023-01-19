#include "pch.h"
#include <omp.h>
#include <mutex>

const std::string Path1 = TESTMODEL_DIR + std::string("/CrossPlane.ply");

class TestHeightMapBySurface : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}

	void generateIncline(std::vector<std::pair<float, Eigen::Vector2f>>& voData)
	{
		for (int i = 0; i < 256; i++)
			for (int k = 0; k < 256; k++)
				voData.emplace_back(std::make_pair((float)i, Eigen::Vector2f(i / 256.0f, k / 256.0f)));
	}

	core::SPoint transPoints(const Point_t& vPoint)
	{
		core::SPoint Point(vPoint.x, vPoint.y, vPoint.z);
		return Point;
	}

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
};

//TEST_F(TestHeightMapBySurface, DT_InValidInput)
//{
//	std::vector<std::pair<float, Eigen::Vector2f>> Data;
//	core::CHeightMapGenerator Generator;
//	EXPECT_FALSE(Generator.generateBySurface(Data, 1, 1));
//	Data.emplace_back(std::make_pair(1.0f, Eigen::Vector2f(0.5f, 0.5f)));
//	EXPECT_FALSE(Generator.generateBySurface(Data, -1, 1));
//}
//
//TEST_F(TestHeightMapBySurface, NT_Incline)
//{
//	core::CHeightMap Map;
//	std::vector<std::pair<float, Eigen::Vector2f>> Data;
//	generateIncline(Data);
//
//	core::CHeightMapGenerator Generator;
//	Generator.generateBySurface(Data, 256, 256);
//	Generator.dumpHeightMap(Map);
//	
//	Eigen::Matrix<float, -1, -1> Image;
//	Image.resize(Map.getWidth(), Map.getHeight());
//	for (int i = 0; i < Map.getWidth(); i++)
//		for (int k = 0; k < Map.getHeight(); k++)
//			Image.coeffRef(i, k) = Map.getValueAt(i, k);
//
//	common::saveImage(Image.cast<int>(), "HeightMap.png");
//}

TEST_F(TestHeightMapBySurface, NT_CrossPlane)
{
	PC_t::Ptr pCloud = loadPC(Path1);
	for (auto& e : *pCloud)
	{
		e.x *= 50;
		e.y *= 50;
		e.z *= 50;
	}

	Eigen::Matrix<core::SPoint, -1, -1> CPs;
	CPs.resize(11, 11);
	for (int i = 0; i < CPs.rows(); i++)
		for (int k = 0; k < CPs.cols(); k++)
		{
			if (i <= 5)
				CPs.coeffRef(i, k) = core::SPoint(i * 50, k * 50, i * 50);
			if (i > 5)
				CPs.coeffRef(i, k) = core::SPoint(i * 50, k * 50, (10 - i) * 50);
		}

	core::CMultilayerSurface Surface(3);
	Surface.setControlPoints(CPs);
	Surface.setIsSaveMesh(true);
	Surface.preCompute();
	
	std::vector<std::pair<float, Eigen::Vector2f>> Data;

	int ThreadSize = 4;
	std::mutex Mutex;

#pragma omp parallel for num_threads(ThreadSize)
	for (int i = 0; i < pCloud->size(); i++)
	{
		auto r = Surface.calcProj(transPoints(pCloud->at(i)));
		//ASSERT_TRUE(r.has_value());

		Mutex.lock();
		Data.emplace_back(std::make_pair(r->_Dist, r->_UV));
		Mutex.unlock();
	}

	core::CHeightMap Map;
	core::CHeightMapGenerator Generator;
	Generator.generateBySurface(Data, 64, 64);
	Generator.dumpHeightMap(Map);

	Eigen::Matrix<float, -1, -1> Image;
	Image.resize(Map.getWidth(), Map.getHeight());
	for (int i = 0; i < Map.getWidth(); i++)
		for (int k = 0; k < Map.getHeight(); k++)
			Image.coeffRef(i, k) = Map.getValueAt(i, k);

	common::saveImage(Image.cast<int>(), "HeightMap.png");
}