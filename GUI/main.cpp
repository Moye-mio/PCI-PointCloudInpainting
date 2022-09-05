#include "GUI.h"
#include <QtWidgets/QApplication>
#include <vtkOutputWindow.h>

int main(int argc, char *argv[])
{
	vtkOutputWindow::SetGlobalWarningDisplay(0);

	QApplication Application(argc, argv);
	CGUI GUI(argc, argv);
	GUI.init();
	GUI.show();

	return Application.exec();
}
