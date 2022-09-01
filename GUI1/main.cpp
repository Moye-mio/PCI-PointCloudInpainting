#include "UI.h"
#include <QtWidgets/QApplication>
#include <QMainWindow>

int main(int argc, char* argv[])
{
	// register global settings
	common::CJsonParser Parser;
	Parser.parseFromFile(definition::CONFIG_FILE_PATH);

	QApplication Application(argc, argv);
	PCLViewer Viewer(argc, argv);

	Viewer.show();

	return Application.exec();
}