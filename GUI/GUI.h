#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_main.h"

class CGUI : public QMainWindow
{
	Q_OBJECT

public:
	CGUI(QWidget *parent = Q_NULLPTR);

private:
	Ui::CMainGUI m_MainGUI;
};
