#include "Creator.h"

using namespace creator;

bool CCreator::createConcave(PC_t::Ptr& voCloud)
{
	for (int i = 0; i < 100; i++)
		for (int k = 0; k < 100; k++)
			voCloud->emplace_back(Point_t(0.0f, i * 1.0f, k * 1.0f));

	for (int i = 0; i < 100; i++)
		for (int k = 0; k < 100; k++)
			voCloud->emplace_back(Point_t(i * 1.0f, k * 1.0f, 0.0f));

	for (int i = 0; i < 100; i++)
		for (int k = 0; k < 100; k++)
			voCloud->emplace_back(Point_t(100.0f, i * 1.0f, k * 1.0f));

	return true;
}
