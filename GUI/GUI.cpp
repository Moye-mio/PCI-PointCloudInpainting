#include "GUI.h"

#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

CGUI::CGUI(QWidget *parent)
	: QMainWindow(parent)
{
	m_MainGUI.setupUi(this);
}
