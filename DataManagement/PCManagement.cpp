#include "pch.h"

#include "PCManagement.h"
#include "PCLoader.h"
#include "PCSaver.h"

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
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<IPCLoader>(Suffix);
	if (pTileLoader)
	{
		std::shared_ptr<CPCWrapper> pPCWrapper = nullptr;
		try
		{
			PC_t::Ptr pData = pTileLoader->loadDataFromFile(vPath);
			int PointSize = pData->size();
			int Size = _msize(pData->data());
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


bool CPCManagement::saveModel(const std::string& vPath)
{
	auto* pSaver = hiveDesignPattern::hiveGetOrCreateProduct<IPCSaver>(hiveUtility::hiveGetFileSuffix(vPath) + "_Save");
	if (pSaver)
	{
		PC_t::Ptr pCloud(new PC_t);
		__mergeAllCloud(pCloud);
		pSaver->saveDataToFileV(*pCloud, vPath);
	}
	else
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to save tile [%1%] due to unknown format.", vPath));
	}
	return true;
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

void CPCManagement::__mergeAllCloud(PC_t::Ptr& voCloud)
{
	for (auto& e : m_CloudMap)
		*voCloud += *e.second->getPointCloud();
}
