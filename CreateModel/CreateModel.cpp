#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

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

PC_t::Ptr compute(const PC_t::Ptr& vLhs, const PC_t::Ptr& vRhs)
{
	PC_t::Ptr pCloud(new PC_t);

	pcl::search::KdTree<Point_t> TreeA, TreeB;
	TreeB.setInputCloud(vRhs);
	float Thres = 0.1f;
	int Count = 0;
	for (int i = 0; i < vLhs->size(); i++)
	{
		std::vector<int> Indices(1);
		std::vector<float> SqrDist(1);

		TreeB.nearestKSearch(vLhs->at(i), 1, Indices, SqrDist);
		if (SqrDist[0] < Thres)
			pCloud->emplace_back(vLhs->at(i));
		else
			Count++;
	}
	std::cout << "Discard: " << Count << std::endl;
	return pCloud;
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

	/*const std::string ModelPath = TESTMODEL_DIR + std::string("/Trimmed/Scene/RAW_Scene_GT.ply");
	PC_t::Ptr pRawCloud = loadPC(ModelPath);
	
	normalizePC(pRawCloud, true);
	square(pRawCloud);

	pcl::io::savePLYFileBinary("RAW_Scene_GT.ply", *pRawCloud);*/

	/*PC_t::Ptr pCloud(new PC_t);
	for (int i = 0; i < 4; i++)
	{
		PC_t::Ptr pTemp = loadPC("Merge/" + std::to_string(i) + ".ply");
		for (const auto& e : *pTemp)
			pCloud->emplace_back(e);
	}

	std::cout << "Total point cloud size " << pCloud->size() << std::endl;
	pcl::io::savePLYFileBinary("Merge/Merge.ply", *pCloud);*/

	const std::string Path = TESTMODEL_DIR + std::string("/Trimmed/Scene/Result/MergeBasedOnGT.ply");
	const std::string Path2 = TESTMODEL_DIR + std::string("/Trimmed/Scene/RAW_Scene_GT.ply");
	PC_t::Ptr pCloud = loadPC(Path);
	PC_t::Ptr pCloudGT = loadPC(Path2);
	auto r = compute(pCloud, pCloudGT);
	std::cout << r->size() << std::endl;
	pcl::io::savePLYFileBinary("r.ply", *r);

	return 0;
}                                                                                                                                                                                                                                                                                                                                