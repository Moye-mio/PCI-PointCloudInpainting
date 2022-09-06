#pragma once

namespace dataManagement
{
	class IPCSaver : public hiveDesignPattern::IProduct
	{
	public:
		IPCSaver() = default;
		~IPCSaver() override = default;

		virtual void saveDataToFileV(const PC_t& vPointCloud, const std::string& vFilePath) = 0;

	private:
	};
}
