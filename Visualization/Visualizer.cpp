#include "Visualizer.h"
#include <pcl/io/ply_io.h>

#include "PointCloudType.h"


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
	PC_t::Ptr cloud(new PC_t); // 创建点云（指针）

	if (pcl::io::loadPLYFile<Point_t>("D:\\Models\\Cloud\\003-004.ply", *cloud) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n"); //文件不存在时，返回错误，终止程序。
		return (-1);
	}

	std::string m_CloudId = "No.1";
	m_pPCLVisualizer->setBackgroundColor(255, 255, 255);
	m_pPCLVisualizer->setBackgroundColor(0, 0, 0);
	m_pPCLVisualizer->addPointCloud<Point_t>(cloud, m_CloudId);
	m_pPCLVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, m_CloudId);

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