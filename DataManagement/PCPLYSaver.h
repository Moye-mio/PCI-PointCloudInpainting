#pragma once
#include "PCSaver.h"

namespace dataManagement
{
	class CPCPLYSaver : public IPCSaver
	{
		void saveDataToFileV(const PC_t& vPointCloud, const std::string& vFilePath) override;
	};
}