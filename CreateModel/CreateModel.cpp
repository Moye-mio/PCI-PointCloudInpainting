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

bool normalizePC(PC_t::Ptr& vCloud, bool vIsChangeCoorYZ = true)
{
	_ASSERTE(vCloud->size());

	PC_t::Ptr pCloud(new PC_t);
	for (const auto& e : *vCloud)
	{
		if (vIsChangeCoorYZ)
			pCloud->emplace_back(Point_t(e.x, e.z, e.y, e.r, e.g, e.b));
		else
			pCloud->emplace_back(Point_t(e.x, e.y, e.z, e.r, e.g, e.b));
	}

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
	}

	vCloud = pCloud;
	return true;
}

bool square(PC_t::Ptr& vCloud)
{
	PC_t::Ptr pCloud(new PC_t);
	Eigen::Vector3f Max(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	for (const auto& e : *vCloud)
	{
		Max[0] = (Max[0] > e.x) ? Max[0] : e.x;
		Max[1] = (Max[1] > e.y) ? Max[1] : e.y;
		Max[2] = (Max[2] > e.z) ? Max[2] : e.z;
	}

	float Diff = std::max(Max[0], Max[1]) - std::min(Max[0], Max[1]);
	for (auto& e : *vCloud)
	{
		if (e.y < Diff / 2 || e.y > Max[1] - Diff / 2) continue;
		pCloud->emplace_back(e);
	}
	vCloud = pCloud;
	return true;
}

int main()
{
	std::uint32_t Mode = 0;
	
	/*CCreator Creator;
	switch (Mode)
	{
	case 0:
		Creator.createConcave(pCloud);

	default:
		break;
	}*/

	const std::string ModelPath = TESTMODEL_DIR + std::string("/Trimmed/Scene/RAW_Scene_GT.ply");
	PC_t::Ptr pRawCloud = loadPC(ModelPath);
	
	normalizePC(pRawCloud, true);
	square(pRawCloud);

	pcl::io::savePLYFileBinary("RAW_Scene_GT.ply", *pRawCloud);

	return 0;
}