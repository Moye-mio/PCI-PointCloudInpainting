#include "Visualizer.h"
#include <pcl/io/ply_io.h>
#include "common/Singleton.h"
#include "PointCloudType.h"
#include "PCManagement.h"


using namespace visualization;

CVisualizer::CVisualizer(int argc, char* argv[])
	: m_pPCLVisualizer(new pcl::visualization::PCLVisualizer(argc, argv, "viewer", pcl::visualization::PCLVisualizerInteractorStyle::New(), false))
{

}

CVisualizer::~CVisualizer()
{

}

bool visualization::CVisualizer::loadModel(const std::string& vPath)
{
	std::string CloudId = "No.1";
	std::string Path = "D:\\Models\\Cloud\\003-004.ply";
	bool r = dataManagement::CPCManagement::getInstance()->loadModel(Path, CloudId);
	if (!r) return false;
	auto pCloud = dataManagement::CPCManagement::getInstance()->getPointCloud(CloudId);

	m_pPCLVisualizer->setBackgroundColor(255, 255, 255);
	m_pPCLVisualizer->setBackgroundColor(0, 0, 0);
	m_pPCLVisualizer->addPointCloud<Point_t>(pCloud->getPointCloud(), CloudId);
	m_pPCLVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, CloudId);

	m_pPCLVisualizer->initCameraParameters();
	m_pPCLVisualizer->resetCamera();
}

//*****************************************************************
//FUNCTION:
void CVisualizer::setInteractor(const vtkSmartPointer<vtkRenderWindowInteractor>& vInteractor, const vtkSmartPointer<vtkRenderWindow>& vRenderWindow)
{
	m_pPCLVisualizer->setupInteractor(vInteractor, vRenderWindow);
	m_pPCLVisualizer->addOrientationMarkerWidgetAxes(vInteractor);
}