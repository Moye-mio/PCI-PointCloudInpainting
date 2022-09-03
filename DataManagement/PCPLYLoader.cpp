#include "pch.h"
#include "PCPLYLoader.h"
#include "DataCommon.h"

using namespace dataManagement;

_REGISTER_EXCLUSIVE_PRODUCT(CPCPLYLoader, PLY_LOADER)

//*****************************************************************
//FUNCTION:
int CPCPLYLoader::__loadDataFromFileV(const std::string& vFileName, PC_t& voPointCloud)
{
	return pcl::io::loadPLYFile<PC_t::PointType>(vFileName, voPointCloud);
}
