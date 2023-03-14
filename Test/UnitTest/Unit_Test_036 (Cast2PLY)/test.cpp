#include "pch.h"

const std::string Path1 = std::string("PoinTr.txt");

bool loadTxT(const std::string& vPath, PC_t::Ptr& vCloud)
{
	std::ifstream TxtStream;
	TxtStream.open(vPath, std::ios::in);
	if (!TxtStream.is_open())
		return false;

	char Buf[1024];
	std::vector<Eigen::Vector3f> Data;
	
	while (TxtStream.getline(Buf, sizeof(Buf)))
	{
		std::string Buffer = Buf;
		std::istringstream IString(Buffer);
		Eigen::Vector3f Pos;

		std::string Temp;
		int Count = 0;
		while (IString >> Temp) {
			if (Count >= 3)
			{
				std::cout << "Float Number >= 3" << std::endl;
				return false;
			}
			Pos[Count++] = std::atof(Temp.c_str());
		}

		Data.emplace_back(Pos);
	}

	for (const auto& e : Data)
		vCloud->emplace_back(Point_t(e.x(), e.y(), e.z()));
	
	std::cout << "Cloud Size is " << vCloud->size() << std::endl;
	return true;
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

	{
		/*for (const auto& e : *pCloud)
			std::cout << e.x << ", " << e.y << ", " << e.z << std::endl;*/
	}

	return pCloud;
}

bool isPointSame(const Point_t& vLhs, const Point_t& vRhs)
{
	if (vLhs.x == vRhs.x && vLhs.y == vRhs.y && vLhs.z == vRhs.z)
		return true;
	else
		return false;
}

bool isPointSimilar(const Point_t& vLhs, const Point_t& vRhs, float vEpsilon)
{
	float Epsilon = vEpsilon;
	if (std::fabsf(vLhs.x - vRhs.x) < Epsilon && std::fabsf(vLhs.y - vRhs.y) < Epsilon && std::fabsf(vLhs.z - vRhs.z) < Epsilon)
		return true;
	else
		return false;
}

void uniform(const std::string& vIn, const std::string& vOut)
{
	PC_t::Ptr pCloud = loadPC(vIn);

	// calc min position
	Eigen::Vector3f Min(FLT_MAX, FLT_MAX, FLT_MAX);
	for (const auto& e : *pCloud)
		for (int i = 0; i < 3; i++)
			Min[i] = (Min[i] > e.getVector3fMap()[i]) ? e.getVector3fMap()[i] : Min[i];

	std::cout << "Min: " << Min[0] << ", " << Min[1] << ", " << Min[2] << std::endl;
	for (auto& e : *pCloud)
	{
		e.x -= Min[0];
		e.y -= Min[1];
		e.z -= Min[2];
	}
	
	pcl::io::savePLYFileBinary(vOut, *pCloud);
}

void extractDiff(const PC_t::Ptr vCloud1, const PC_t::Ptr vCloud2, const std::string& vOut, float vThres)
{
	int Count = 0;
	PC_t::Ptr pDiff(new PC_t);
	for (int i = 0; i < vCloud1->size(); i++)
	{
		const auto& p = vCloud1->at(i);
		bool HasSame = false;
		for (const auto& e : *vCloud2)
			if (isPointSimilar(p, e, vThres))
			{
				HasSame = true;
				Count++;
				break;
			}
		if (HasSame == false)
			pDiff->emplace_back(p);
	}
	std::cout << "Same Count: " << Count << std::endl;
	pcl::io::savePLYFileBinary(vOut, *pDiff);

}

void merge(const PC_t::Ptr& vCloud1, const PC_t::Ptr& vCloud2, const std::string& vOut)
{
	PC_t::Ptr pMerge(new PC_t);
	for (const auto& e : *vCloud1)
		pMerge->emplace_back(Point_t(e.x, e.y, e.z, (std::uint8_t)0, (std::uint8_t)0, (std::uint8_t)0, (std::uint8_t)255));

	for (const auto& e : *vCloud2)
		pMerge->emplace_back(Point_t(e.x, e.y, e.z, (std::uint8_t)0, (std::uint8_t)0, (std::uint8_t)0, (std::uint8_t)255));

	pcl::io::savePLYFileBinary(vOut, *pMerge);
}

TEST(TypeCastTest, Txt2PLY) 
{
	/*PC_t::Ptr pCloud(new PC_t);
	EXPECT_TRUE(loadTxT(Path1, pCloud));
	pcl::io::savePLYFileBinary("Save.ply", *pCloud);*/

	
	/*std::string Type = "Hole";
	std::string InPath = TESTMODEL_DIR + std::string("\\ExperimentResult\\") + Type + std::string("\\Sub\\ASCII\\Terrain_5_") + Type + std::string("_Sub_8_ASCII.ply");
	std::string OutPath = TESTMODEL_DIR + std::string("\\ExperimentResult\\") + Type + std::string("\\Sub\\Uniform\\Terrain_5_") + Type + std::string("_Sub_8_Uniform.ply");
	uniform(InPath, OutPath);*/

	/*PC_t::Ptr pCloud1 = loadPC("D:\\Projects\\PCI\\Models\\ExperimentResult\\MeshLab\\Sub\\Uniform\\Terrain_5_MeshLab_Sub_8_Uniform.ply");
	PC_t::Ptr pCloud2 = loadPC("D:\\Projects\\PCI\\Models\\ExperimentResult\\Hole\\Sub\\Uniform\\Terrain_5_Hole_Sub_8_Uniform.ply");
	std::string OutPath = "D:\\Projects\\PCI\\Models\\ExperimentResult\\MeshLab\\Sub\\Result\\Terrain_5_MeshLab_Sub_8_Result.ply";
	extractDiff(pCloud1, pCloud2, OutPath, 300.0f);*/

	PC_t::Ptr pCloud1 = loadPC("D:\\Projects\\PCI\\Models\\ExperimentResult\\MeshLab\\Sub\\Result\\Terrain_5_MeshLab_Sub_8_Result.ply");
	PC_t::Ptr pCloud2 = loadPC("D:\\Projects\\PCI\\Models\\ExperimentResult\\Hole\\Sub\\Uniform\\Terrain_5_Hole_Sub_8_Uniform.ply");
	std::string OutPath = "D:\\Projects\\PCI\\Models\\ExperimentResult\\MeshLab\\Sub\\Merge\\Terrain_5_MeshLab_Sub_8_Merge.ply";
	merge(pCloud1, pCloud2, OutPath);
}


