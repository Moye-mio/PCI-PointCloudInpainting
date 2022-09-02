#include "GUI.h"

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

CGUI::CGUI(int argc, char* argv[], QWidget *vParent)
	: QMainWindow(vParent)
	, m_pMainGUI(new Ui::CMainGUI)
	, m_pVisualizer(new visualization::CVisualizer(argc, argv))
{
	m_pMainGUI->setupUi(this);

	__initWidget();
}

CGUI::~CGUI()
{
	m_pVisualizer = nullptr;
}

void CGUI::__initWidget()
{
	m_pMainGUI->CVTKWidget->SetRenderWindow(m_pVisualizer->getRenderWindow());
	m_pVisualizer->setInteractor(m_pMainGUI->CVTKWidget->GetInteractor(), m_pMainGUI->CVTKWidget->GetRenderWindow());
}
