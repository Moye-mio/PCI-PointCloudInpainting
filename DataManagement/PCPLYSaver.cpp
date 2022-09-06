#include "pch.h"
#include "PCPLYSaver.h"
#include "DataCommon.h"

using namespace dataManagement;

_REGISTER_EXCLUSIVE_PRODUCT(CPCPLYSaver, PLY_SAVER);

//*****************************************************************
//FUNCTION: 
void CPCPLYSaver::saveDataToFileV(const PC_t& vPointCloud, const std::string& vFilePath)
{
	if (pcl::io::savePLYFileBinary<PC_t::PointType>(vFilePath, vPointCloud) == -1)
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to write file [%1%] due to incorrect file path.", vFilePath));
}