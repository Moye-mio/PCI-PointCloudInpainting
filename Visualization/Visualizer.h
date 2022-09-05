#pragma once
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>

namespace visualization
{
	class CVisualizer
	{
	public:
		CVisualizer(int argc, char* argv[]);
		~CVisualizer();
		
		[[nodiscard]] bool loadModel(const std::string& vPath);
		[[nodiscard]] vtkSmartPointer<vtkRenderWindow> getRenderWindow() const { return m_pPCLVisualizer->getRenderWindow(); }
		void setInteractor(const vtkSmartPointer<vtkRenderWindowInteractor>& vInteractor, const vtkSmartPointer<vtkRenderWindow>& vRenderWindow);
		void unloadModel();
		void refresh();

	private:
		pcl::visualization::PCLVisualizer::Ptr m_pPCLVisualizer;

	};
}