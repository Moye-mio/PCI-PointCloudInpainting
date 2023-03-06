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

TEST(TypeCastTest, Txt2PLY) 
{
	PC_t::Ptr pCloud(new PC_t);
	EXPECT_TRUE(loadTxT(Path1, pCloud));
	pcl::io::savePLYFileBinary("Save.ply", *pCloud);
}
