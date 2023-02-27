#pragma once

#include "Vertex.h"

namespace core
{
	class CNormalSampler
	{
	public:
		CNormalSampler() {}
		~CNormalSampler() {}

		[[nodiscard]] bool setData(const Eigen::Matrix<SVertex, -1, -1>& vData);

		std::optional<Eigen::Vector3f> sample(const Eigen::Vector2f& vUV);

	private:
		bool __isUVValid(const Eigen::Vector2f& vUV);

	private:
		Eigen::Matrix<SVertex, -1, -1> m_Vertices;

	};
}