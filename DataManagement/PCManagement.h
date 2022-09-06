#pragma once
#include "PC.h"

namespace dataManagement
{
	class CPCManagement : public hiveDesignPattern::CSingleton<CPCManagement>
	{
	public:
		~CPCManagement();

		[[nodiscard]] bool loadModel(const std::string& vPath, const std::string& vId);
		[[nodiscard]] bool saveModel(const std::string& vPath);
		
		const std::shared_ptr<CPCWrapper> getPointCloud(const std::string& vId) const;

	private:
		CPCManagement() {};

		void __mergeAllCloud(PC_t::Ptr& voCloud);

	private:
		std::unordered_map<std::string, std::shared_ptr<CPCWrapper>> m_CloudMap;

	friend class hiveDesignPattern::CSingleton<CPCManagement>;
	};
}