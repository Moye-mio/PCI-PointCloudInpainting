#pragma once

#include "PointCloudType.h"


namespace dataManagement
{
	class IPointCloudLoader : public hiveDesignPattern::IProduct
	{
	public:
		IPointCloudLoader() = default;
		~IPointCloudLoader() override = default;

		PC_t::Ptr loadDataFromFile(const std::string& vFileName);

	private:
		virtual int __loadDataFromFileV(const std::string& vFileName, PC_t& voPointCloud) = 0;
	};
}