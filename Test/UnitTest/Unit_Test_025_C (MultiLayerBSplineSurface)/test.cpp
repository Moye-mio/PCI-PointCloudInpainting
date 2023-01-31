#include "pch.h"

#include "HeightMapGenerator.h"
#include "Image.h"
#include "Surface2CloudMapper.h"

const std::string ModelPath = TESTMODEL_DIR + std::string("/Trimmed/Concave/GT_Concave.ply");

class TestMultiLayerSurface_C : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
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

	void generateConcaveCP(Eigen::Matrix<core::SPoint, -1, -1>& voCPs, int vCoef = 1)
	{
		voCPs.resize(18, 8);

		std::vector<core::SPoint> Points;
		for (int i = -1; i <= 6; i++)
		{
			for (int k = 6; k >= 0; k--)
				Points.emplace_back(core::SPoint(0.0f, i * vCoef, k * vCoef));

			for (int k = 1; k <= 5; k++)
				Points.emplace_back(core::SPoint(k * vCoef, i * vCoef, 0.0f));

			for (int k = 1; k <= 6; k++)
				Points.emplace_back(core::SPoint(5 * vCoef, i * vCoef, k * vCoef));
		}

		int Number = 0;
		for (int k = 0; k < voCPs.cols(); k++)
			for (int i = 0; i < voCPs.rows(); i++)
				voCPs.coeffRef(i, k) = Points[Number++];
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

	void extractProjData(core::CMultilayerSurface* vSurface, const PC_t::Ptr& vCloud, std::vector<std::pair<float, Eigen::Vector2f>>& voData)
	{
		voData.clear();

		for (int i = 0; i < vCloud->size(); i++)
		{
			Point_t Point = vCloud->at(i);
			auto r = vSurface->calcProj(core::SPoint(Point.x, Point.y, Point.z));
			voData.emplace_back(std::make_pair(r->_Dist, r->_UV));
		}
	}
};

TEST_F(TestMultiLayerSurface_C, NT_SinglePoint)
{
	Eigen::Matrix<core::SPoint, -1, -1> CPs;

	core::SPoint InitPoint(0, 20, 0);

	generateConcaveCP(CPs, 20);
	core::CMultilayerSurface* pSurface(new core::CMultilayerSurface(3));
	pSurface->setControlPoints(CPs);
	pSurface->setSubLayer(3);
	pSurface->setIsSaveMesh(true);
	pSurface->preCompute();
	auto r = pSurface->calcProj(InitPoint);
	EXPECT_TRUE(r.has_value());
	
	std::vector<std::pair<float, Eigen::Vector2f>> Data;
	if (r.has_value())
	{
		const auto p = r->_Point;
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("Dist: [%1%]", r->_Dist));
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR3("Proj Vertex: [%1%, %2%, %3%]", p.x(), p.y(), p.z()));
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("UV: [%1%, %2%]", r->_UV[0], r->_UV[1]));

		PC_t::Ptr pCloud(new PC_t);
		pCloud->emplace_back(Point_t(InitPoint.x(), InitPoint.y(), InitPoint.z(), (std::uint8_t)255, 0, 0, 0));
		pCloud->emplace_back(Point_t(p.x(), p.y(), p.z(), 0, 0, (std::uint8_t)255, 0));

		pcl::io::savePLYFileBinary("Output.ply", *pCloud);
		Data.emplace_back(std::make_pair(r->_Dist, r->_UV));
	}

	core::CHeightMap Map, Mask, Inpainted, InpaintedMask;
	core::CHeightMapGenerator Generator;
	Generator.generateBySurface(Data, 36, 16);
	Generator.dumpHeightMap(Map);
	Map.generateMask(Mask);
	saveMap2Image(Map, "Map.png", 30);
	saveMap2Image(Mask, "Mask.png", 255);
	
	for (int i = 0; i < Mask.getWidth(); i++)
		for (int k = 0; k < Mask.getHeight(); k++)
		{
			if (Mask.getValueAt(i, k) == 0) Mask.setValueAt(1, i, k);
			else if (Mask.getValueAt(i, k) == 1) Mask.setValueAt(0, i, k);
		}

	PC_t::Ptr pGTCloud = loadPC(ModelPath);
	std::vector<std::pair<float, Eigen::Vector2f>> Data2;
	extractProjData(pSurface, pGTCloud, Data2);
	core::CHeightMapGenerator Generator2;
	Generator2.generateBySurface(Data2, 36, 16);
	Generator2.dumpHeightMap(Inpainted);
	Inpainted.generateMask(InpaintedMask);
	saveMap2Image(Inpainted, "Inpainted.png", 30);
	saveMap2Image(InpaintedMask, "InpaintedMask.png", 255);

	std::shared_ptr<core::CMultilayerSurface> pTrSurface(pSurface);
	PC_t::Ptr pNewCloud(new PC_t);
	dataManagement::CSurface2CloudMapper Mapper;
	EXPECT_TRUE(Mapper.setSurface(pTrSurface));
	EXPECT_TRUE(Mapper.map2Cloud(Map, Mask, Inpainted, 5));
	Mapper.dumpCloud(pNewCloud);
	pcl::io::savePLYFileBinary("NewCloud.ply", *pNewCloud);
}
