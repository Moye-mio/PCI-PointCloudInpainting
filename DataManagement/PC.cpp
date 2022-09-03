#include "pch.h"

#include "PC.h"

using namespace dataManagement;

CPCWrapper::CPCWrapper(const PC_t::Ptr& vPC, const std::string& vId)
	: m_pPC(vPC)
	, m_Id(vId)
{

}

CPCWrapper::~CPCWrapper()
{
	m_pPC = nullptr;
}
