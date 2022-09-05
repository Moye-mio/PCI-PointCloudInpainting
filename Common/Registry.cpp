#include "Registry.h"
#include <iostream>

#define SET_PROP(KEY, VAL, CONTAINER)																								\
if ((CONTAINER).count(KEY) != 0)																									\
{																																	\
	std::cout << "Update Property [" << (KEY) << "] from \"" << (CONTAINER)[KEY] << "\"to \"" << (VAL) << "\"." << std::endl;		\
}																																	\
else																																\
{																																	\
	std::cout << "Append Property [" << (KEY) << "]: " << (VAL) << "." << std::endl;												\
}																																	\
(CONTAINER)[KEY] = VAL;

#define GET_PROP(KEY, VAL, CONTAINER)									\
if ((CONTAINER).count(KEY) == 0)										\
{																		\
	std::cout << "Property [" << (KEY) << "] not found." << std::endl;	\
	return false;														\
}																		\
(VAL) = (CONTAINER).at(KEY);											\
return true;															\


void common::CRegistry::setProperty(const std::string& vKey, int vValue)
{
	SET_PROP(vKey, vValue, m_IntPropertyMap)
}

void common::CRegistry::setProperty(const std::string& vKey, double vValue)
{
	SET_PROP(vKey, vValue, m_DoublePropertyMap)
}

void common::CRegistry::setProperty(const std::string& vKey, bool vValue)
{
	SET_PROP(vKey, vValue, m_BoolPropertyMap)
}

void common::CRegistry::setProperty(const std::string& vKey, const std::string& vValue)
{
	SET_PROP(vKey, vValue, m_StringPropertyMap)
}

void common::CRegistry::setProperty(const std::string& vKey, const char* vValue)
{
	setProperty(vKey, std::string(vValue));
}

bool common::CRegistry::getProperty(const std::string& vKey, int& vValue) const
{
	GET_PROP(vKey, vValue, m_IntPropertyMap)
}

bool common::CRegistry::getProperty(const std::string& vKey, double& vValue) const
{
	GET_PROP(vKey, vValue, m_DoublePropertyMap)
}

bool common::CRegistry::getProperty(const std::string& vKey, bool& vValue) const
{
	GET_PROP(vKey, vValue, m_BoolPropertyMap)
}

bool common::CRegistry::getProperty(const std::string& vKey, std::string& vValue) const
{
	GET_PROP(vKey, vValue, m_StringPropertyMap)
}

void common::CRegistry::reset()
{
	m_IntPropertyMap.clear();
	m_DoublePropertyMap.clear();
	m_BoolPropertyMap.clear();
	m_StringPropertyMap.clear();
}

