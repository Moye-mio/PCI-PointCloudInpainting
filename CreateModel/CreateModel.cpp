#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Creator.h"

using namespace creator;

int main()
{
	std::uint32_t Mode = 0;
	
	PC_t::Ptr pCloud(new PC_t);
	CCreator Creator;
	switch (Mode)
	{
	case 0:
		Creator.createConcave(pCloud);

	default:
		break;
	}

	pcl::io::savePLYFileBinary("Concave.ply", *pCloud);

	return 0;
}