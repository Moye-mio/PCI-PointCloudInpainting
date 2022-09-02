#include "GUI.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication Application(argc, argv);
	CGUI GUI(argc, argv);
	GUI.show();
	return Application.exec();
}
