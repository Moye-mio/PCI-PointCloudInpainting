#include "JsonParser.h"
#include "Registry.h"
#include <fstream>
#include <iostream>

common::EParseStatus common::CJsonParser::parseFromFile(const std::string& vFileName)
{
	std::ifstream Stream(vFileName);
	if (!Stream.is_open())
	{
		std::cout << "File not exists." << std::endl;
		return EParseStatus::FileNotFound;
	}
	Stream >> m_Configuration;
	Stream.close();

	return __registerConfiguration();
}

common::EParseStatus common::CJsonParser::__registerConfiguration() const
{
	for (const auto& Item : m_Configuration.items())
	{
		switch (Item.value().type())
		{
		case nlohmann::detail::value_t::number_integer:
		case nlohmann::detail::value_t::number_unsigned:
			SET_PROPERTY(Item.key(), static_cast<int>(Item.value())); break;
		case nlohmann::detail::value_t::number_float:
			SET_PROPERTY(Item.key(), static_cast<double>(Item.value())); break;
		case nlohmann::detail::value_t::boolean:
			SET_PROPERTY(Item.key(), static_cast<bool>(Item.value())); break;
		case nlohmann::detail::value_t::string:
			SET_PROPERTY(Item.key(), static_cast<std::string>(Item.value())); break;
		default:
			std::cout << "Not supported type." << std::endl;
			return EParseStatus::TypeNotSupported;
		}
	}

	return EParseStatus::Successful;
}

void common::checkParseStatus(EParseStatus vParseStatus)
{
	if (vParseStatus != EParseStatus::FileNotFound)
	{
		std::cout << "Parse json file failed." << std::endl;
		exit(-1);
	}
}
