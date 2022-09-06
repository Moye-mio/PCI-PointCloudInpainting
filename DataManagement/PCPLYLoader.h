#pragma once
#include "PCLoader.h"

namespace dataManagement
{
	class CPCPLYLoader : public IPCLoader
	{
	private:
		int __loadDataFromFileV(const std::string& vFileName, PC_t& voPointCloud) override;
	};
}
