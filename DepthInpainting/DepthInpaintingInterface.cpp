#include "pch.h"
#include "DepthInpaintingInterface.h"

using namespace depthInpainting;

void depthInpainting::runDepthInpaiting(const PC_t::Ptr& vCloud, PC_t::Ptr& voResultCloud)
{
	_ASSERTE(vCloud != nullptr);
	_ASSERTE(vCloud->size());

	core::CHeightMap HeightMap;
	core::CHeightMapGenerator HGenerator;
	HGenerator.setCloud(vCloud);
	HGenerator.dumpHeightMap(HeightMap);
	_ASSERTE(HeightMap.isValid());

	core::CGradientMap GradientMap;
	core::CGradientMapGenerator GGenerator;
	GGenerator.generate(HeightMap, true);
	GGenerator.dumpGradientMap(GradientMap);
	_ASSERTE(GradientMap.isValid());

	core::CHeightMap Mask;
	GradientMap.generateMask(Mask);
	_ASSERTE(Mask.isValid());




}

