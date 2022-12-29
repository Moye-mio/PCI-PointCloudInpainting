#pragma once
#include "BSplineSurface.h"

namespace core
{
	class CMultiLayerBSplineSurface : CBSplineSurface
	{
	public:
		
		float calcProj(const SPoint& vPoint, Eigen::Vector2f& vUV);

	private:
		void __generateMultiLayerNodes();
		bool __isMultiLayerReady();

	private:
		int m_Sub = 5;
		int m_Layers = 3;

		std::vector<Eigen::Matrix<SPoint, -1, -1>> m_MultiLayerNodes;
	};
}