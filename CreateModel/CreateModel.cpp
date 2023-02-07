#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Creator.h"

using namespace creator;

PC_t::Ptr loadPC(const std::string& vPath)
{
	PC_t::Ptr pCloud(new PC_t);
	int r = pcl::io::loadPLYFile<Point_t>(vPath, *pCloud);
	_ASSERTE(r != -1);
	_ASSERTE(pCloud->size());
	std::cout << "Model Point Size: " << pCloud->size() << std::endl;
	return pCloud;
}

int main()
{
	std::uint32_t Mode = 0;
	
	PC_t::Ptr pCloud(new PC_t);
	/*CCreator Creator;
	switch (Mode)
	{
	case 0:
		Creator.createConcave(pCloud);

	default:
		break;
	}*/

	const std::string ModelPath = TESTMODEL_DIR + std::string("/Trimmed/Scene/WH_Scene.ply");
	PC_t::Ptr pCloud2 = loadPC(ModelPath);
	/*for (const auto& e : *pCloud2)
		pCloud->emplace_back(Point_t(e.x, e.y, e.z, e.r, e.g, e.b));

	Eigen::Vector3f Min(FLT_MAX, FLT_MAX, FLT_MAX);

	for (const auto& e : *pCloud)
	{
		Min[0] = (Min[0] < e.x) ? Min[0] : e.x;
		Min[1] = (Min[1] < e.y) ? Min[1] : e.y;
		Min[2] = (Min[2] < e.z) ? Min[2] : e.z;
	}

	for (auto& e : *pCloud)
	{
		e.x -= Min[0];
		e.y -= Min[1];
		e.z -= Min[2];
	}*/
	Eigen::Vector3f Max(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	for (const auto& e : *pCloud2)
	{
		Max[0] = (Max[0] > e.x) ? Max[0] : e.x;
		Max[1] = (Max[1] > e.y) ? Max[1] : e.y;
		Max[2] = (Max[2] > e.z) ? Max[2] : e.z;
	}

	float Diff = std::max(Max[0], Max[1]) - std::min(Max[0], Max[1]);
	for (auto& e : *pCloud2)
	{
		if (e.y < Diff / 2 || e.y > Max[1] - Diff / 2) continue;
		pCloud->emplace_back(e);
	}

	pcl::io::savePLYFileBinary("Scene.ply", *pCloud);

	return 0;
}