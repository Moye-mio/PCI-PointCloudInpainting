#pragma once

#include "HeightMap.h"
#include "GradientMap.h"

namespace core
{
	class CMapSaver
	{
	public:
		CMapSaver() = default;
		~CMapSaver() = default;

		bool save(const core::CHeightMap& vMap, const std::string& vPath, int vCoef = 1);
		bool save(const core::CGradientMap& vMap, const std::string& vFileName, int vCoef = 1);

	};
}