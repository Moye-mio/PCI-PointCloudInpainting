#include "pch.h"


class TestMap2Cloud : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{
	}
};

TEST_F(TestMap2Cloud, DT_Nothing)
{
	core::CHeightMap2PCMapper Mapper;
	PC_t::Ptr pCloud;
	std::pair<core::CHeightMap, core::CHeightMap> Maps;
	core::SAABB Box;
	ASSERT_DEATH(Mapper.map2PC(pCloud, Maps, Box, -1), ".*");
}

TEST_F(TestMap2Cloud, NT_)
{
	core::CHeightMap2PCMapper Mapper;
	PC_t::Ptr pCloud;

	core::CHeightMap RawMap, FilledMap;
	Eigen::MatrixXf Filled(3, 3);
	Filled << 0, 0, 0,
		10, 10, 10,
		20, 20, 20;

	RawMap.setSize(Filled.rows(), Filled.cols());
	FilledMap.setHeightMap(Filled);
	auto Maps = std::make_pair(RawMap, FilledMap);

	core::SAABB Box;
	Box._Max = Eigen::Vector3f(20, 20, 20);
	Box._Min = Eigen::Vector3f(0, 0, 0);

	Mapper.map2PC(pCloud, Maps, Box, 10);

	pcl::io::savePLYFileBinary<Point_t>("1.ply", *pCloud);
}