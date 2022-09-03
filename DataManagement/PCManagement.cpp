#include "pch.h"

#include "PCManagement.h"
#include "PCLoader.h"


using namespace dataManagement;

CPCManagement::~CPCManagement()
{

}

bool CPCManagement::loadModel(const std::string& vPath, const std::string& vId)
{
	std::string LowerFileName = hiveUtility::hiveLocateFile(vPath);
	if (LowerFileName.empty())
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to load file [%1%] because it does not exist.", vPath));
		return false;
	}
	auto Suffix = hiveUtility::hiveGetFileSuffix(vPath);
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<IPointCloudLoader>(Suffix);
	if (pTileLoader)
	{
		std::shared_ptr<CPCWrapper> pPCWrapper = nullptr;
		try
		{
			PC_t::Ptr pData = pTileLoader->loadDataFromFile(vPath);
			pPCWrapper = std::make_shared<CPCWrapper>(pData, vId);
		}
		catch (...) {}

		if (!pPCWrapper) return false;

		m_CloudMap.emplace(vId, pPCWrapper);
		return true;
	}
	else
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to load tile [%1%] due to unknown format.", vPath));
		return false;
	}
}

const std::shared_ptr<CPCWrapper> CPCManagement::getPointCloud(const std::string& vId) const
{
	auto Iter = m_CloudMap.find(vId);
	if (Iter != m_CloudMap.end())
	{
		return Iter->second;
	}
	else
		return nullptr;
}

