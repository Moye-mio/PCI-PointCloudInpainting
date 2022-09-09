#include "pch.h"

#include "AABBEstimation.h"
#include "AABB.h"

using namespace core;

CAABBEstimation::CAABBEstimation(const PC_t::Ptr& vCloud)
	: m_pCloud(vCloud)
{ }

SAABB CAABBEstimation::compute()
{
	_ASSERTE(m_pCloud->size());

	SAABB AABB(Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX), Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX));
	for (auto& e : *m_pCloud)
	{
		for (int i = 0; i < AABB._Max.size(); i++)
		{
			AABB._Max[i] = (e.getVector3fMap()[i] > AABB._Max[i]) ? e.getVector3fMap()[i] : AABB._Max[i];
			AABB._Min[i] = (e.getVector3fMap()[i] < AABB._Min[i]) ? e.getVector3fMap()[i] : AABB._Min[i];
		}
	}

	_ASSERTE(AABB.isValid());

	return AABB;
}

