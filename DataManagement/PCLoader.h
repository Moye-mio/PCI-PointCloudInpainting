#pragma once

namespace dataManagement
{
	class IPCLoader : public hiveDesignPattern::IProduct
	{
	public:
		IPCLoader() = default;
		~IPCLoader() override = default;

		PC_t::Ptr loadDataFromFile(const std::string& vFileName);

	private:
		virtual int __loadDataFromFileV(const std::string& vFileName, PC_t& voPointCloud) = 0;
	};
}