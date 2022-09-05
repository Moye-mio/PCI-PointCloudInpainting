#pragma once
#include "common/Singleton.h"
#include "common/DesignPatternInterface.h"
#include <unordered_map>

namespace common
{
	class CRegistry : public hiveDesignPattern::CSingleton<CRegistry>
	{
	public:
		~CRegistry() override {}
		CRegistry(const CRegistry&) = delete;
		CRegistry(CRegistry&&) = delete;
		CRegistry& operator=(const CRegistry&) = delete;
		CRegistry& operator=(CRegistry&&) = delete;

		void setProperty(const std::string& vKey, int vValue);
		void setProperty(const std::string& vKey, double vValue);
		void setProperty(const std::string& vKey, bool vValue);
		void setProperty(const std::string& vKey, const std::string& vValue);
		void setProperty(const std::string& vKey, const char* vValue);

		bool getProperty(const std::string& vKey, int& vValue) const;
		bool getProperty(const std::string& vKey, double& vValue) const;
		bool getProperty(const std::string& vKey, bool& vValue) const;
		bool getProperty(const std::string& vKey, std::string& vValue) const;

		void reset();

	private:
		CRegistry() {}

		std::unordered_map<std::string, int> m_IntPropertyMap;
		std::unordered_map<std::string, double> m_DoublePropertyMap;
		std::unordered_map<std::string, bool> m_BoolPropertyMap;
		std::unordered_map<std::string, std::string> m_StringPropertyMap;

		friend class hiveDesignPattern::CSingleton<CRegistry>;
	};
}

#define GET_PROPERTY(KEY, VALUE) \
	common::CRegistry::getInstance()->getProperty(KEY, VALUE)

#define SET_PROPERTY(KEY, VALUE) \
	common::CRegistry::getInstance()->setProperty(KEY, VALUE)