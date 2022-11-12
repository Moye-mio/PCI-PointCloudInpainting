#pragma once
#include "AABB.h"

namespace core
{
	class CExtremePoint
	{
	public:
		CExtremePoint(const PC_t::Ptr& vCloud);
		~CExtremePoint() = default;

		[[nodiscard]] bool setAABB(const SAABB& vBox);
		[[nodiscard]] bool compute();
		[[nodiscard]] bool computeExtreme();
		void dumpIndices(std::vector<int>& voIndices) const { voIndices = m_ExtremeId; }
		void dumpPoints(std::vector<Point_t>& voPoints) const;
		void dumpExtremeIndices(std::vector<int>& voIndices) const { voIndices = m_ExtremePointId; }
		void dumpExtremePoints(std::vector<Point_t>& voPoints) const;

	private:
		bool __preprocess();

	private:
		std::vector<int> m_ExtremeId;
		std::vector<int> m_ExtremePointId;

		PC_t::Ptr m_pCloud;
		SAABB m_Box;
	};
}