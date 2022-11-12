#include "pch.h"
#include "PCLASLoader.h"
#include "DataCommon.h"

using namespace dataManagement;

_REGISTER_EXCLUSIVE_PRODUCT(CPCLASLoader, LAS_LOADER)

//*****************************************************************
//FUNCTION:
int CPCLASLoader::__loadDataFromFileV(const std::string& vFileName, PC_t& voPointCloud)
{
	std::ifstream Ifs(vFileName, std::ios::in | std::ios::binary);
	if (!Ifs.good()) return false;
	liblas::ReaderFactory Factory;
	liblas::Reader Reader = Factory.CreateWithStream(Ifs);
	liblas::Header LasHeader = Reader.GetHeader();
	int PointsNumber = LasHeader.GetPointRecordsCount();

	voPointCloud.width = PointsNumber;
	voPointCloud.height = 1;
	voPointCloud.is_dense = false;

	for (int i = 0; i < PointsNumber; i++)
	{
		Reader.ReadNextPoint();
		const liblas::Point& Point = Reader.GetPoint();
		liblas::Classification Classification = Point.GetClassification();
		std::bitset<8> Bitset = Classification.GetFlags();

		if (Bitset == std::bitset<8>() || Bitset[1] == 1 && Bitset[2] == 0) //alltypes or ground  bitset_type[0] == 1 && bitset_type[1] == 0 && bitset_type[2] == 1
			voPointCloud.points.push_back(Point_t((float)Point.GetX(), (float)Point.GetY(), (float)Point.GetZ()));
	}
	Ifs.close();
	return 0;
}
