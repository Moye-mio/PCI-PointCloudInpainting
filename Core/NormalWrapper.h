#pragma once
#include "Vertex.h"

namespace core
{
	class CNormalWrapper
	{
	public:
		CNormalWrapper() = default;
		~CNormalWrapper() = default;

		bool compute(const std::vector<SVertex>& vVertices, float vRadius, std::vector<Eigen::Vector3f>& voNormals);

	private:
		bool __transVertex2PCLPoint(const std::vector<SVertex>& vVertices, PC_t::Ptr& voCloud);
		bool __transPCLNormal2Normal(const NormalPC_t::Ptr& vNormals, std::vector<Eigen::Vector3f>& voNormals);

	};
}