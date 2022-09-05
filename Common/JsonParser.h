#pragma once
#include <json.hpp>

namespace common
{
	using Json = nlohmann::json;

	enum class EParseStatus
	{
		FileNotFound,
		TypeNotSupported,
		Successful
	};

	class CJsonParser
	{
	public:
		EParseStatus parseFromFile(const std::string& vFileName);

	private:
		Json m_Configuration;

		EParseStatus __registerConfiguration() const;
	};

	void checkParseStatus(EParseStatus vParseStatus);
}
